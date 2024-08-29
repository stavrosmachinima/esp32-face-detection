#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "WiFi.h"
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include "fd_forward.h"  // face detection

// XREIAZETAI ESP VERSION 1.0.6 GIA NA PAI3EI EPEIDH DEN EXEI TO FD_FORWARD.H

// Replace with your network credentials
const char *ssid = "";
const char *password = "";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
mtmn_config_t mtmn_config = { 0 };
int bytesPerPixel = 3;  // this is in RGB888 format for face-detection

enum FaceColor {
  RED = 0xFF0000,
  GREEN = 0x00FF00,
  BLUE = 0x0000FF,
  YELLOW = 0xFFFF00,
  BLACK = 0x000000,
  WHITE = 0xFFFFFF
};

void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id = 2) {
  FaceColor color;
  switch (face_id) {
    case 0:
      color = RED;
      break;
    case 1:
      color = GREEN;
      break;
    case 2:
      color = BLUE;
      break;
    case 3:
      color = YELLOW;
      break;
    case 4:
      color = BLACK;
      break;
    case 5:
      color = WHITE;
      break;
    default:
      color = BLUE;
      break;  
  }
  if (boxes->len > 0) {

    int width = image_matrix->w;
    int height = image_matrix->h;
    int x1 = (int)boxes->box[0].box_p[0];
    int y1 = (int)boxes->box[0].box_p[1];
    int x2 = (int)boxes->box[0].box_p[2];
    int y2 = (int)boxes->box[0].box_p[3];

    for (int x = x1; x <= x2; x++) {
      image_matrix->item[(y1 * width + x) * bytesPerPixel] = color;  // Top line
      image_matrix->item[(y2 * width + x) * bytesPerPixel] = color;  // Bottom line
    }

    for (int y = y1; y <= y2; y++) {
      image_matrix->item[(y * width + x1) * bytesPerPixel] = color;  // Left line
      image_matrix->item[(y * width + x2) * bytesPerPixel] = color;  // Right line
    }
  }
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
    return res;


  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, bytesPerPixel);
      if (!image_matrix) {
        Serial.println("dl_matrix3du_alloc failed");
        esp_camera_fb_return(fb);
        res = ESP_FAIL;
      } else {
        if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
          Serial.println("fmt2rgb888 failed");
          res = ESP_FAIL;
        } else {
          box_array_t *boxes = face_detect(image_matrix, &mtmn_config);
          if (boxes != NULL) {
            Serial.printf("Face detected! Number of faces: %d\n", boxes->len);
            draw_face_boxes(image_matrix, boxes);

            if (!fmt2jpg(image_matrix->item, fb->width * fb->height * bytesPerPixel, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)) {
              Serial.println("fmt2jpg failed");
              res = ESP_FAIL;
            }

          } else {
            Serial.println("No face detected");
            if (fb->format != PIXFORMAT_JPEG) {
              Serial.println("Format was not jpeg. Formatting again now..");
              bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
              if (!jpeg_converted) {
                Serial.println("JPEG compression failed");
                res = ESP_FAIL;
              }
            } else {
              _jpg_buf_len = fb->len;
              _jpg_buf = fb->buf;
            }
          }
        }
        dl_matrix3du_free(image_matrix);
      }
    }

    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);

    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
      break;
    delay(1000);
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&camera_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(camera_httpd, &index_uri);
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_240X240;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  // modifications tou sensora
  // Isws xreiazetai na valeis ton parakatw kwdika pio panw h katw gia na pai3ei
  /*
  sensor_t * s = esp_camera_sensor_get();
  if (s) {
      s->set_framesize(s, FRAMESIZE_QVGA);
      s->set_quality(s, 10);
      s->set_brightness(s, 0);     // -2 to 2
      s->set_contrast(s, 0);       // -2 to 2
      s->set_saturation(s, 0);     // -2 to 2
      s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
      s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
      s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
      s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
      s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
      s->set_aec2(s, 0);           // 0 = disable , 1 = enable
      s->set_ae_level(s, 0);       // -2 to 2
      s->set_aec_value(s, 300);    // 0 to 1200
      s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
      s->set_agc_gain(s, 0);       // 0 to 30
      s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
      s->set_bpc(s, 1);            // 0 = disable , 1 = enable
      s->set_wpc(s, 1);            // 0 = disable , 1 = enable
      s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
      s->set_lenc(s, 1);           // 0 = disable , 1 = enable
      s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
      s->set_vflip(s, 0);          // 0 = disable , 1 = enable
      s->set_dcw(s, 1);            // 0 = disable , 1 = enable
      s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  }
  */

  // Wi-Fi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());

  // Start streaming web server
  startCameraServer();
}

void loop() {
}
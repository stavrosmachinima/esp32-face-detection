# esp32-face-detection-minimal

## Minimal esp32-camera face-detection Arduino project

I first used the web-server in examples in Arduino IDE but face-detection didn't work...
Then, I found out the expanded web-server in 3.x branch, there is face-detection but said my chip had too small storage space...
So, I created my **OWN**.

It also differs with other projects that in this case, I create my own streaming configuration, sensor configuration, face detection configuration and my own drawings of the face.

Fake Person created with the tool [Url for creating faces](https://this-person-does-not-exist.com/en)

![Face Identification of a Woman](https://github.com/user-attachments/assets/43f9b1b2-e9ae-427a-aed4-0bf9957915a0)


Example Output


![Serial Output example in Arduino IDE](https://github.com/user-attachments/assets/920e3993-ca65-48f6-b099-6891a0b96899)

It can also detect multiple faces at once if one changes the configuration.

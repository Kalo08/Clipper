CLIPPER


This project uses a simple robotic arm driven by servos and controlled by a vr headset and controllers. it connects to the headset to feed real time data back to the servos to manipulate the arm. the arm has hair clippers attached, and can be used to cut hair. The headset communicates to a rasberry pi using UDP and sends servo commands. The pi drives the servos through its gpio ports and the servos have power through a simple battery pack and bread board.

https://www.youtube.com/watch?v=h8sehnFcGIc&authuser=0

Heres a YouTube Video explaining how everything works!

<img width="6048" height="8064" alt="clipper" src="https://github.com/user-attachments/assets/206e67c2-b5a2-40d2-a7ae-669efc0123d3" />

<img width="5712" height="4284" alt="IMG_8768" src="https://github.com/user-attachments/assets/4bfcc626-359d-40ab-8699-c69da485dbc8" />

Here are pictures of the finalized arm! It has the a base that allows it to rotate, move up and down, and stay attached ot teh table. That connects to the elbow which allows us to move the clippers even more. 

Wiring: The Pi is powered and connects to the servos. The Pi communicates to our computer. The computer facilitates it comunication with the VR head set. When the VR headset communicates to the Pi, and this tells the pi to activates the servos. The code tells the servos which ones need to go to keep in sync with the VR.


| Item | Qty | Unit Price (USD) | Line Total (USD) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| Raspberry Pi 4 (4GB) | 1 | 55.00 | 55.00 | MSRP; 2GB ~$45 / 8GB ~$75 |
| SG90 Micro Servo | 3 | 4.00 | 12.00 | ~$3-5 each; often cheaper in multipacks |
| Hair Trimmer | 1 | 25.00 | 25.00 | Price provided |
| Aluminum Extrusion (2020 T-slot 1m) | 1 | 12.00 | 12.00 | Varies by profile and length |
| Battery Pack (USB power bank) | 1 | 15.00 | 15.00 | ~10000mAh; Pi 4 needs 5V/3A output |
| Breadboard (830-point) | 1 | 5.00 | 5.00 | Half-size ~$3 |
| Jumper Wires (120pc M-M/M-F/F-F kit) | 1 | 6.00 | 6.00 | Mixed kit |
| Total |  |  | 130.00 |  |

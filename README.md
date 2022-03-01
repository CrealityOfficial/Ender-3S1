# Ender-3S1 Firmware

Creality, dedicating to bringing benefits to human beings via technology innovations, has received support from both users and 3D printing enthusiasts. With gratefulness, Creality wants to continue the pace of making the world a better place with you all. This time, Creality will open the source code and we believe GitHub is the way to go. 

This is the repository that contains the source code and the development versions of the firmware running on the Creality [Ender-3 S1](https://www.creality.com/goods-detail/creality-ender-3-s1-3d-printer). It's based on the well-known Marlin but with modifications.

The firmware for the Creality Ender-3S1 is proudly based on Marlin2.0 byScott Lahteine (@thinkyhead), Roxanne Neufeld (@Roxy-3D), Chris Pepper (@p3p), Bob Kuhn (@Bob-the-Kuhn), João Brazio (@jbrazio), Erik van der Zalm (@ErikZalm) and is distributed under the terms of the GNU GPL 3 license.

If you want to download the latest firmware version, go to Releases page and download the needed files. In the [releases](https://github.com/CrealityOfficial/Ender-3S1) page you will find the source code and the SD Files needed for the LCD Display. After that, normally you need to update the SD files of the display and gradually complete the updates of menus, graphics and functionalities. 

Please refer to: [YouTube](https://youtu.be/B4egner2JMM)
In order to get instructions on how to flash the firmware and load new LCD SD files to the display. 


# New Features
1. Repair the bug of slow printing speed with Cloud APP.
2. Repair the bug of losing gcode file while resume printing.
3. Repair the bug of slow speed when printing threading.
4. Repair the bug that the parameter setting of "Control-Motion-Jerk " cannot be saved.
5. Repair the bug that Z-axis acceleration cannot be set to 100-200m/s2.
7. Repair the bug that the interface of auto leveling function doesn’t exit when CR-touch is abnormal.
8. Repair the bug that the computer port sends the “Pause” (M25) command but the display doesn’t synchronize show the “Continue printing” page when online printing.

# Special Point
A special structure on Ender 3S1 we must introduce is expansion port, which can fulfill more possibility, especially support CV-Laser module.

How to start CV-Laser function?

Four steps: Check-Install-Use-Engrave

## Check firmware version
When using the laser module on Ender-3 S1 for the first time, please check if the motherboard firmware and screen firmware are the applicable versions respectively. 
Please connect the machine to power supply, turn on the power switch on the side, select "Control" - "Info", to check if the firmware version is V1.0.4 or above.

## How to install CrealityPrint software
1、Please download “Creality Print” to install in [Creality official website](https://www.creality.com/download)

2、Double click to open the software installation package, click "Next-"Accept".

3、Confirm the installation location of the software, click "Browse"-"Next"- "Install"-"Finish".

## How to use CrealityPrint software
1、Add Ender 3S1 into software.

2、Make sure the current status is “Laser”.

3、Click on the "Picture" button on the left and find the image file you need to engrave. Open to import the picture into the CrealityPrint software.

4、Adjust the position, size, rotation angle and other parameters of the file in the right-hand console, setting the processing mode and working parameters, finally preview the processing effect of it.

5、Finally, click on "Generate G-Code" to save the engraving file on the SD card.

## How to engrave
1、Insert SD card into Ender 3S1 and turn on the power switch on the side.

2、After Ender 3S1 is switched on, if you are using it for the first time, please select laser engrave.

### Notice: 
1、To switch from fusing to laser engrave, select "Contror - "Switched Laser engraving"

2、The software pops up the following prompt box. Please ensure that the laser module is installed correctly and select "Confirm". ( Please note that Laser module must be installed and disassembled only under power off)

3、Place the engraving material under the laser head,counter-clockwise push the fixed focus bar, adjust the laser focus to the appropriate value through the knob (different thickness of the material corresponding to different values), until the focusing bar touches the engraved material surface when natural vertical down. Finally, clockwise push the focusing bar until it is attached by the magnet, then select "Finish”.
Return to the main interface, select "Auto Home", wait for the machine back to zero completed.

4、Select "Engrave" and press the knob to confirm. Select "Run Range", the laser head will move repeatedly around the maximum X and Y edges. You can adjust the engraving material to the right position.

5、Press the “Directly Engrave” to start engraving. 

### Notice:
When engraving or cutting thin objects (e.g. paper), the laser may penetrate the object and leave marks. It is recommended to put a flat object which laser cannot penetrate, such as aluminium or stainless steel plates, before you put the engraving material.

You can develop more functions through the source code, such as water cooling, CNC and other functions.

# Issues and Suggestions
Your feedback is very important to us, as it helps us improve even faster. Please test this firmware and let us know if it misbehaves in any way. We are standing by!

In order to get responses in an efficient way, we recommend you to follow some guidelines:

1、First of all, search for related issues.

2、Detail the firmware version you're running.

3、Explain to us the error or bug, so that we can test it properly.

4、In the title, indicate the label of the issue. (For example: #issue)

# Development Process
The code is currently in development, trying to improve functionalities.
Since it’s possible for the **advanced users** to contribute in firmware development, we suppose you know the points even if they have not been clearly illustrated by Creality.

The master branch is stable and it's currently of the version 2.0.x. The master branch stores code created by Creality. Once a release is done, the users, get to upgrade the version and give feedback to us. We get to know the bugs as well as optimization based on the feedback from you and Creality will make a decision on what to be included into the master branch. 

By integrating suggested improvements, we will make a branch from the version.

This is a classic code development process and we want more, so we really want you to participate from the very beginning.

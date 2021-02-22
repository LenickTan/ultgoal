/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="intaketesting", group="Iterative Opmode")
public class intaketesting extends OpMode

{
    double intakePower = 0;     //power for intake
    double pulleyPower = 0;     //power for pulley

    public DcMotor intake; //power intake
    public DcMotor pulley;      //pulley to lift for shooter

    //servos----------------------------------
    public Servo pinwheel1;       //right intake to straighten ring
    public Servo pinwheel2;       //left intake to straighten ring
    public Servo pinwheel3;       // bottom pinwheel???
    public Servo pinwheel4;

    public Servo flicker; //pushes ring to shooter

    @Override
    public void init() {

        pinwheel1 = hardwareMap.servo.get("pinwheel1");
        pinwheel2 = hardwareMap.servo.get("pinwheel2");
       pinwheel3 = hardwareMap.servo.get("pinwheel3");
        pinwheel4 = hardwareMap.servo.get("pinwheel4");

        intake = hardwareMap.dcMotor.get("intake");

        flicker = hardwareMap.servo.get("flicker");

        pulley = hardwareMap.dcMotor.get("pulley");



    }
    @Override
    public void loop() {

        //GAMEPAD 1 ------------------------------------------

        //INTAKE--------------------------------------------

        //intake pinwheels (moves in opposite directions)
        if((gamepad1.right_trigger) >0.1) {
            pinwheel2.setPosition(pinwheel2.getPosition() + 0.1);
            pinwheel3.setPosition(pinwheel3.getPosition() + 0.1);
            pinwheel1.setPosition(pinwheel1.getPosition() - 0.1);
            pinwheel4.setPosition(pinwheel4.getPosition() - 0.1);

        } else if ((gamepad1.left_trigger) >0.1) {
            pinwheel2.setPosition(pinwheel2.getPosition() - 0.1);
            pinwheel3.setPosition(pinwheel3.getPosition() - 0.1);
            pinwheel1.setPosition(pinwheel1.getPosition() + 0.1);
            pinwheel4.setPosition(pinwheel4.getPosition() + 0.1);

        } else {
            pinwheel2.setPosition(0.5);
            pinwheel3.setPosition(0.5);
            pinwheel1.setPosition(0.5);
            pinwheel4.setPosition(0.5);
        }
        //powers intake
        if (Math.abs(gamepad1.left_trigger) > .1) {
            intakePower = -1;

        } else if (Math.abs(gamepad1.right_trigger) > .1) {
            intakePower = 1;

        } else {
            intakePower = 0;
        }

        //flicker moves back and forth 90 degrees
        if ((gamepad1.right_bumper) ) {
            flicker.setPosition(flicker.getPosition() + 0.1);
        } else if ((gamepad1.left_bumper) ) {
            flicker.setPosition(flicker.getPosition() - 0.1);
        } else {
            flicker.setPosition(0.5);
        }
        intake.setPower(intakePower);

        if (Math.abs(gamepad2.left_trigger) > .1) {
            pulleyPower = -1;

        } else if (Math.abs(gamepad2.right_trigger) > .1) {
            pulleyPower = 1;

        } else {
            pulleyPower = 0;

        }



        pulley.setPower(pulleyPower*0.15);

        intake.setPower(intakePower);

    }
}


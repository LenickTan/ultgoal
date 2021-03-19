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

@TeleOp(name="wobble", group="Iterative Opmode")
public class wobble extends OpMode {
    boolean changed = false;
    public Servo lift;  //wobble goal lifter
    public Servo lift2;
    public Servo lift3;
    public Servo flicker; //pushes ring to shooter

    public Servo claw; //wobble goal clasp/claw


    @Override
    public void init() {

        lift = hardwareMap.servo.get("lift");
        lift2 = hardwareMap.servo.get("lift2");
        lift3 = hardwareMap.servo.get("lift3");

        claw = hardwareMap.servo.get("claw");

        flicker = hardwareMap.servo.get("flicker");
    }

    @Override
    public void loop() {
        //GAMEPAD 1 ------------------------------------------
/*
        //wobble goal
        //WOBBLE GOAL-------------------------------
        //41 degrees???
        //servo goes all the way up
       if (gamepad2.a) {
            lift.setPosition(1);
            //lift2.setPosition(1);
            //lift3.setPosition(1);
        }

        //180 degrees
        //servo keeps going down
        if (gamepad2.b) {
            lift.setPosition(0);
            //lift2.setPosition(0.5);
            //lift3.setPosition(0.5);
        }

        //back to set position
        //servo went all the way up?
        if (gamepad2.x) {
            lift.setPosition(0.5);
            //lift2.setPosition(0);
            //lift3.setPosition(0);
        }
*/


       if((gamepad1.right_trigger) >0.1) {
           lift.setPosition(lift.getPosition() + 0.1);
            lift2.setPosition(lift2.getPosition() + 0.1);
            lift3.setPosition(lift3.getPosition() - 0.1);

        } else if ((gamepad1.left_trigger) >0.1) {
            lift.setPosition(lift.getPosition() - 0.1);
            lift2.setPosition(lift.getPosition() - 0.1);
            lift3.setPosition(lift3.getPosition() + 0.1);


        } else {
            lift.setPosition(0.5);
            lift2.setPosition(0.5);
            lift3.setPosition(0.5);

        }

        //open close claw
        if (gamepad1.y && !changed) {
            if(claw.getPosition() ==0 )
            claw.setPosition(0.99);
            else {
                claw.setPosition(0);
            }
            changed = true;
        } else if (!gamepad1.y) {
            changed = false;
        }

        //flicker moves back and forth 90 degrees continuous??????
        if ((gamepad1.right_bumper) ) {
            flicker.setPosition(flicker.getPosition() + 0.1);
        } else if ((gamepad1.left_bumper) ) {
            flicker.setPosition(flicker.getPosition() - 0.1);
        } else {
            flicker.setPosition(0.5);
        }

    }

    }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Ultrasonic {
    //declaring I2C object (ultrasonic sensor)
    private static final byte address = 112;
    private I2C ultrasonic;
    private Timer ultrasonicTimer = new Timer();
    private byte[] data = new byte[2];

    public Ultrasonic() {
        ultrasonic = new I2C(I2C.Port.kOnboard, address);
        ultrasonic.write(224, 81);
        ultrasonicTimer.reset();
        ultrasonicTimer.start();
    }
    
    //returns distance in meters (ultrasonic returns in cm, so divided by 100)
    public double getDistance() {
        //condition checks if it has been 100 milliseconds since the write to the I2C port has been done (datasheet recommends 80 ms)
        if (canRead()) {
            ultrasonic.read(225, 2, data); //reads data into the byte array data
            int highByte = data[0]; //highByte = first byte of 2 bytes returned from the read
            int lowByte = data[1]; //lowByte = second byte of 2 bytes returned from the read
            //values of the lowbyte greater than 127 overflow to -128 or greater, so negative values of lowbyte need to be adjusted
            if (lowByte < 0) lowByte += 256;
            double distance = ((256 * highByte) + lowByte) / 100.0; //shifting highByte 8 bits to the left, adding lowbyte, converting to m
            //SmartDashboard.putNumber("Distance: ", distance); //putting data on smart dashboard
            ultrasonic.write(224, 81); //doing another reading
            ultrasonicTimer.reset(); //resetting the timer
            return distance;
        }
        return -1;
    }

    public boolean canRead() {
        return ultrasonicTimer.get() > 0.1;
    }
}
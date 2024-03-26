// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.OccupancyState;
import frc.robot.subsystems.Superstructure.AimingState;

public class LED extends SubsystemBase {
  private AddressableLED leds = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(88);

  private static Supplier<IntakeState> mIntakeStateSupplier;
  private static Supplier<OccupancyState> mOccupancySupplier;
  private static Supplier<Superstructure.AimingState> mShooterAimingState;
  private static Supplier<Drivetrain.AimingState> mDrivetrainAimingState;
  
  public LED(Supplier<IntakeState> intakeStateSupplier, Supplier<OccupancyState> occupancySupplier, Supplier<Superstructure.AimingState> pivotAimingSupplier, Supplier<Drivetrain.AimingState> driveAimSupplier) {
    leds.setLength(buffer.getLength());
    leds.start();
    mIntakeStateSupplier = intakeStateSupplier;
    mOccupancySupplier = occupancySupplier;
    mShooterAimingState = pivotAimingSupplier;
    mDrivetrainAimingState = driveAimSupplier;
  }

  public void setElevatorLED(int R, int G, int B) {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, R, G, B);
    }
    leds.setData(buffer);
  }

  public void setAllLED(int R, int G, int B) {

    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, R, G, B);
    }
    leds.setData(buffer);
  }

  @Override
  public void periodic() {

    if (mOccupancySupplier.get() == OccupancyState.kFull) {
      if (mShooterAimingState.get() == AimingState.kTooFar) {
      setAllLED(255, 255, 255);
    } else if (mShooterAimingState.get() == AimingState.kAimed && mDrivetrainAimingState.get() == Drivetrain.AimingState.kAimed) {
      setAllLED(0, 255, 0);
    } else if ((mShooterAimingState.get() == AimingState.kAiming || (mShooterAimingState.get() == AimingState.kAimed && mDrivetrainAimingState.get() == Drivetrain.AimingState.kNotAiming)) || (mDrivetrainAimingState.get() == Drivetrain.AimingState.kAiming || (mDrivetrainAimingState.get() == Drivetrain.AimingState.kAimed && mShooterAimingState.get() == AimingState.kNotAiming))){
      setAllLED(255, 0, 0);
    } else {
      setAllLED(255, 50, 0);
    }
    } else if (mOccupancySupplier.get() == OccupancyState.kPartialPickup) {
      setAllLED(255, 255, 0);
    } else {
      setAllLED(0,0, 255);
    }
  }


  // public Command setLedsOccupied() {
  //   return this.runEnd(()->setAllLED(0,0,255), ()->setAllLED(255,50,0));
  // }

  // public Command setLedsPurple() {
  //   return this.runEnd(()->setAllLED(255,0,255), ()->setAllLED(0,0,255));
  // }

}

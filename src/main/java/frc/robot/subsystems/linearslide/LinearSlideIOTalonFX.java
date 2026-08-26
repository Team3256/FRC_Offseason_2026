package frc.robot.subsystems.linearslide;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.PhoenixUtil;

public class LinearSlideIOTalonFX implements LinearSlideIO {
    private final TalonFX slideMotor = new TalonFX(LinearSlideConstants.slideMotorID);

    private final MotionMagicVoltage motionMagicRequest =
        new MotionMagicVoltage(0).withSlot(0).withEnableFOC(LinearSlideConstants.kUseFOC);

    private final StatusSignal<Voltage> motorVoltage = slideMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> velocity = slideMotor.getVelocity();
    private final StatusSignal<Angle> position = slideMotor.getPosition();
    private final StatusSignal<Current> statorCurrent = slideMotor.getStatorCurrent();
    private final StatusSignal<Current> supplyCurrent = slideMotor.getSupplyCurrent();

    public LinearSlideIOTalonFX() {
        PhoenixUtil.applyMotorConfigs(
            slideMotor, LinearSlideConstants.motorConfigs, LinearSlideConstants.flashConfigRetries);

        BaseStatusSignal.setUpdateFrequencyForAll(
            LinearSlideConstants.updateFrequency,
            motorVoltage,
            velocity,
            position,
            statorCurrent,
            supplyCurrent);

        PhoenixUtil.registerSignals(
            false, motorVoltage, velocity, position, statorCurrent, supplyCurrent);
    }



}



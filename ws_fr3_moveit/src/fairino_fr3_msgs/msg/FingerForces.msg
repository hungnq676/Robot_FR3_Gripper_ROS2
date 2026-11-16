# Readings of the two jaw force sensors (RFP602 thin-film FSR, one per jaw,
# under a 20 x 20 mm pad ~10 mm from the fingertip). Published by the Gazebo
# emulator in simulation and by the sensor board on the real gripper.
std_msgs/Header header

# Normal force on each pad, N (calibrated; FSRs are only ~+-10-25 % accurate,
# use for contact detection and force thresholds, not precise measurement).
float32 force_clamp_1
float32 force_clamp_2

# Raw ADC counts of the FSR voltage divider (12-bit), for debugging/calibration.
uint16 raw_clamp_1
uint16 raw_clamp_2

# Force above the contact threshold.
bool contact_clamp_1
bool contact_clamp_2

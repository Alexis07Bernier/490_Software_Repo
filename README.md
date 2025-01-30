# 490_Software_Repo
## PuckSensorApp.m is Ali's previous code

## HockeyPuckFilter.m
### How It Works
#### Load Sensor Data
* Reads accelerometer and gyroscope data from an Excel file.
* Computes time step (Δ𝑡) for processing.

#### Initialize Filters
* Uses MadgwickAHRS to estimate quaternions.
* Sets up Kalman filter parameters (state, covariance, noise matrices).
* Process Data Using Madgwick + Kalman Filter
* Updates the Madgwick filter with accelerometer and gyroscope data.
* Converts acceleration from the local frame to the global frame using quaternions.
* Removes gravity to get net motion acceleration.
* Uses a Kalman filter to estimate velocity while reducing noise.

#### Plot Velocity Results
* Plots the estimated velocity components over time.

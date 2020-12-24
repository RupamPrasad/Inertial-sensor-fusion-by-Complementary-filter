close all;
clear;
%% connection between matlab and arduino uno
a = arduino('COM3', 'Uno', 'Libraries', 'I2C');
%% creating mpu6050 object and setting its parameters
fs = 50; % Sample Rate in Hz   
imu = mpu6050(a,'SampleRate',fs,'SamplesPerRead',5,'ReadMode','Latest');

GyroscopeNoise = 3.0462e-06; % GyroscopeNoise (variance) in units of rad/s

AccelerometerNoise = 0.0061; % AccelerometerNoise (variance) in units of m/s^2

%% for visualisation
viewer = HelperOrientationViewer('Title',{'IMU Filter'});

%%

stopTimer=1000;
tic;
dt=1/50;
gyroX=0;
gyroY=0;
gyroZ=0;
totX=0;
totY=0;

while(toc < stopTimer)
    [accelReadings,timestamp] = readAcceleration(imu);
    [gyro,timestamp] = readAngularVelocity(imu);
    angleX=atan((accelReadings(2)-AccelerometerNoise)/(sqrt((accelReadings(1)-AccelerometerNoise)^2+(accelReadings(3)-AccelerometerNoise)^2)));
    angleY=atan(-(accelReadings(1)-AccelerometerNoise)/(sqrt((accelReadings(2)-AccelerometerNoise)^2+(accelReadings(3)--AccelerometerNoise)^2)));
    gyroX=(gyro(1)-GyroscopeNoise)*dt;
    gyroY=(gyro(2)-GyroscopeNoise)*dt;
    gyroZ=gyroZ+((gyro(3)-GyroscopeNoise)*dt);
    
    totX=(0.98*(totX+gyroX))+(0.02*angleX);
    totY=(0.98*(totY+gyroY))+(0.02*angleY);
    
    quat = eul2quat([gyroZ totY totX]);
    quatScalar = quaternion(quat);
    viewer(quatScalar);  
%     eul = 180*quat2eul(quat)/pi; %% returns quaternion TO euler angles
%     disp([angleX angleY]);
end
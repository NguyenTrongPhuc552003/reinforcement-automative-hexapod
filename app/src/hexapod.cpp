#include "hexapod.hpp"
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <sys/time.h>

const float Hexapod::OBSTACLE_THRESHOLD = 20.0f; // cm
const float Hexapod::TURN_SPEED = 1.0f;
const float Hexapod::WALK_SPEED = 2.0f;

Hexapod::Hexapod() : pca1(PCA9685::ADDR_1), pca2(PCA9685::ADDR_2) {}

Hexapod::~Hexapod() {}

bool Hexapod::init()
{
    printf("Initializing Hexapod...\n");

    // Initialize PCA9685 controllers
    if (!pca1.init())
    {
        printf("Failed to initialize PCA9685 #1\n");
        return false;
    }

    if (!pca2.init())
    {
        printf("Failed to initialize PCA9685 #2\n");
        return false;
    }

    // Initialize ultrasonic sensor
    if (!ultrasonic.init())
    {
        printf("Failed to initialize ultrasonic sensor\n");
        return false;
    }

    // Center all servos
    for (int leg = 0; leg < 6; leg++)
    {
        setLegAngles(leg, Angles(0, 45, -90)); // Safe standing position
    }

    printf("Hexapod initialized successfully!\n");
    return true;
}

void Hexapod::run()
{
    printf("Starting autonomous mode...\n");

    struct timeval lastTime, currentTime;
    gettimeofday(&lastTime, nullptr);

    gait.setSpeed(WALK_SPEED);
    gait.setDirection(0); // Forward

    while (true)
    {
        gettimeofday(&currentTime, nullptr);
        float dt = (currentTime.tv_sec - lastTime.tv_sec) +
                   (currentTime.tv_usec - lastTime.tv_usec) / 1000000.0f;
        lastTime = currentTime;

        // Check for obstacles
        obstacleAvoidance();

        // Update gait
        gait.update(dt);

        // Update servo positions
        updateServos();

        usleep(50000); // 50ms update rate
    }
}

void Hexapod::updateServos()
{
    for (int leg = 0; leg < 6; leg++)
    {
        Point3D legPos = gait.getLegPosition(leg);
        Angles angles = Kinematics::inverseKinematics(legPos);
        setLegAngles(leg, angles);
    }
}

void Hexapod::setLegAngles(int leg, const Angles &angles)
{
    int coxaChannel = getServoChannel(leg, 0);
    int femurChannel = getServoChannel(leg, 1);
    int tibiaChannel = getServoChannel(leg, 2);

    // Clamp angles to servo limits
    float coxaAngle = fmaxf(-90, fminf(90, angles.coxa));
    float femurAngle = fmaxf(-90, fminf(90, angles.femur));
    float tibiaAngle = fmaxf(-90, fminf(90, angles.tibia));

    if (coxaChannel < 16)
    {
        pca1.setServoAngle(coxaChannel, coxaAngle);
    }
    else
    {
        pca2.setServoAngle(coxaChannel - 16, coxaAngle);
    }

    if (femurChannel < 16)
    {
        pca1.setServoAngle(femurChannel, femurAngle);
    }
    else
    {
        pca2.setServoAngle(femurChannel - 16, femurAngle);
    }

    if (tibiaChannel < 16)
    {
        pca1.setServoAngle(tibiaChannel, tibiaAngle);
    }
    else
    {
        pca2.setServoAngle(tibiaChannel - 16, tibiaAngle);
    }
}

void Hexapod::obstacleAvoidance()
{
    static int turnCounter = 0;

    float distance = ultrasonic.getDistance();

    if (distance > 0 && distance < OBSTACLE_THRESHOLD)
    {
        printf("Obstacle detected at %.1f cm - turning right\n", distance);

        // Stop forward movement and turn right
        gait.setSpeed(TURN_SPEED);
        gait.setDirection(M_PI / 2); // 90° right

        turnCounter = 20; // Turn for ~1 second
    }
    else if (turnCounter > 0)
    {
        turnCounter--;
        if (turnCounter == 0)
        {
            printf("Resuming forward movement\n");
            gait.setSpeed(WALK_SPEED);
            gait.setDirection(0); // Forward
        }
    }
}

int Hexapod::getServoChannel(int leg, int joint)
{
    // Servo mapping: 18 servos across 2 PCA9685 boards
    // Leg order: Front-Right, Front-Left, Mid-Right, Mid-Left, Rear-Right, Rear-Left
    // Joint order: Coxa, Femur, Tibia

    static const int channels[6][3] = {
        {0, 1, 2},    // Leg 0: Front-Right
        {3, 4, 5},    // Leg 1: Front-Left
        {6, 7, 8},    // Leg 2: Mid-Right
        {9, 10, 11},  // Leg 3: Mid-Left
        {12, 13, 14}, // Leg 4: Rear-Right
        {15, 16, 17}  // Leg 5: Rear-Left (channels 16,17 on PCA2)
    };

    return channels[leg][joint];
}

#include "asservissement.h"

#define FIR_ORDER 5
#define VALID_RADIUS 5
#define VALID_ANGLE 0.05

double initialAngleError = 0;

double leftMotorValues[FIR_ORDER] = {0};
double rightMotorValues[FIR_ORDER] = {0};

void setMotorsSmoothly() {
	double left = 0, right = 0;
	for (int i=0; i<FIR_ORDER; i++) {
		left += leftMotorValues[i];
		right += rightMotorValues[i];
	}
	setMotors(left/FIR_ORDER, right/FIR_ORDER);
}

void setSmooth(double left, double right) {
	for (int i=0; i<FIR_ORDER-1; i++) {
		leftMotorValues[i] = leftMotorValues[i+1];
		rightMotorValues[i] = rightMotorValues[i+1];
	}
	leftMotorValues[FIR_ORDER-1] = left;
	rightMotorValues[FIR_ORDER-1] = right;
}

void smoothReset() {
	for (int i=0; i<FIR_ORDER; i++) {
		leftMotorValues[i] = 0;
		rightMotorValues[i] = 0;
	}
}

double speedCurve(double progress) {
    double speed = -4*progress*(progress-1);
    return (speed<0)?0:speed;
}

double toZeroTwoPi(double angle) {
    return fmod(fmod(angle, M_TWOPI) + M_TWOPI, M_TWOPI);
}

double optimalAngle(double angle) { // anciennement toPiPi
    return toZeroTwoPi(angle + M_PI) - M_PI;
}

double toZeroOne(double x) {
	if (x < 0) return 0;
	if (x > 1) return 1;
	return x;
}

void ASSERV_update(CONSIGNE point, CONSIGNE previous) {
    if (indexStrategie >= nb_points || !match_started) {
    	stopMoteurs();
    	return;
    }

    ODO_odometrie();

    // calculs préliminaires
    double pointDistance = sqrt(pow(point.x - g_x, 2) + pow(point.y - g_y, 2));
    double segmentLength = sqrt(pow(point.x - previous.x, 2) + pow(point.y - previous.y, 2));
    double moveProgress = toZeroOne(1 - pointDistance/(segmentLength + VALID_RADIUS));

    double pointDirection = atan2(point.y - g_y, point.x - g_x);
    double angleError = optimalAngle( pointDirection - g_angle - ((point.dir==BACKWARD)?M_PI:0) );
    if (isnan(initialAngleError)) initialAngleError = angleError;
    double rotateProgress = toZeroOne(1 - angleError/initialAngleError);

    // vitesse du robot

    if (pointDistance > VALID_RADIUS) {
        // le robot est loin
    	double speed = 0, spin = 0;
    	speed = VMAX_DISTANCE*speedCurve(moveProgress)*segmentLength/1000 + VMIN_DISTANCE;
    	speed *= 1/(1+pow(fabs(angleError/VALID_ANGLE), 2));
    	speed *= (point.dir==FORWARD)?1:-1;
    	spin = VMAX_ROTATION*(speedCurve(rotateProgress)*fabs(initialAngleError)/M_PI + speedCurve(moveProgress)*fabs(angleError))/2;
    	spin += VMIN_ROTATION*(1-rotateProgress);
    	spin *= -angleError/fabs(angleError);// * (1-moveProgress);
    	setSmooth(speed - spin, speed + spin);
    } else {
        // le robot est arrivé au point visé
    	actionneursUpdate(point);
    	indexStrategie++;
    	initialAngleError = NAN;
    	if (indexStrategie == nb_points) {
    		smoothReset();
    	} else {
    		setSmooth(0, 0);
    	}
    }
    setMotorsSmoothly();
}

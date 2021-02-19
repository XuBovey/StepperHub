#!/usr/bin/env python


"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
    mm                         |<----84---->|
                               |<---60--->| |
                               |____      | |
                           J3 /o____|=o=o=__+__   _____
                             / / )d        | |       | lenPump 78
                            / /           _|o|_   ___|_
                           / /           (x,y,z)
                          / /
                         / /  J2 to J3 250, smallArm
                        / /
                       / /
                      / /
                     /o/ )c     <-------J2------———————
                     | |                              | J1 to J2 218 big arm
                     | |                              |
                     | |                              |
                     | |                              |
                     | |                              |
            _________| |                              |
           |          o| )b     <-------J1------———————
        J0 |o )a       |                              | J1 to base 10
  (((((((((|o|))))))))))))))                          |
           |o|                                        |
  (((((((((|o|))))))))))))))    _......Base.....______|_

            |         |
            |<--102-->| J0 to J1

a,b,c,d: jointAngle
(x,y,z): endpoint postion
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

from math import acos, isnan, pi, atan2, sqrt, sin, cos

MATH_TRANS          = 180/pi
MATH_LOWER_ARM      = 218
MATH_UPPER_ARM      = 250

class robot_arm_ik():
    # def __init__(self):

    def postion_to_angle(self, postion, angle): # postion (mm), angle (degree)
        x = postion[0]
        y = postion[1]
        z = postion[2]

        if y == 0:
            theta1_degree = 0
        elif y < 0:
            theta1_degree = - atan2(x,y)*MATH_TRANS
        elif y > 0:
            theta1_degree = atan2(x,y)*MATH_TRANS
        
        r_offset = 100 + 95 # mm 95.3/94 = motor2[DH_a], 39.22=suctionCup_DummyB[x]-auxMotor2[x]
        z_offset = 100 - 70# mm 71=(317.44-246.12)# 103 = motor2[y] = motor1[y]+motor2[DH_d], (317.44-246.12)=auxMotor2[y]-suctionCup_DummyB[y]

        r = sqrt(x**2 + y**2) - r_offset # X
        s = z - z_offset    # Y

        if r < 0:
            print("r={:}, s={:}".format(r,s))
            return False
        
        if sqrt(s ** 2 + r ** 2) > (MATH_LOWER_ARM + MATH_UPPER_ARM):
            print("too far to move")
            return False

        D = (s ** 2 + r ** 2 - MATH_LOWER_ARM ** 2 - MATH_UPPER_ARM ** 2) / (2 * MATH_LOWER_ARM * MATH_UPPER_ARM)
        theta3 = acos(D)
        theta2 = atan2(s,r) + atan2(MATH_UPPER_ARM*sin(theta3),MATH_LOWER_ARM + MATH_UPPER_ARM*cos(theta3))

        theta2_degree = theta2*MATH_TRANS
        theta3_degree = theta3*MATH_TRANS
        # print("theta1={:f}, theta2={:f}, theta3={:f}".format(theta1_degree, theta2_degree, theta3_degree))    

        angle[0] = theta1_degree
        angle[1] = theta2_degree
        angle[2] = theta3_degree
        return True

    def armAngle_to_motorAngle(self, angle):
        return (angle[0], angle[1], angle[1] - angle[2])
        # return (angle[0], angle[1], angle[2])
        # motor[0] = angle[0] # motor 1
        # motor[1] = angle[1] # motor 2
        # motor[2] = angle[2] - angle[1] # motor 3

if __name__=="__main__":
    try:
        ik = robot_arm_ik()
        postion=[426.35,146.80,253.52]
        angle=[0,0,0]
        motor=[0,0,0]
        ik.robot_arm_ik(postion,angle)
        print("angle={:.2f},{:.2f}, {:.2f}".format(angle[0], angle[1], angle[2]))    
    except Exception as e:
        print(e)

    finally:
        print("quit")




















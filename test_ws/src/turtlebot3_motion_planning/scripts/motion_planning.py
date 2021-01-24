#! /usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

x_or = 0.0
y_or = 0.0
z_imu = 0.0

robot_gps_latitude = 0.0
robot_gps_longitude = 0.0

robot_gps_latitude_memory = 0.0
robot_gps_longitude_memory = 0.0



def odom_callback(msg):
    rate = rospy.Rate(0.5)
    global x_or
    global y_or
    # go = Odometry() is not needed
    print ("------------------------------------------------")
    print ("pose x = " + str(msg.pose.pose.position.x))
    print ("pose y = " + str(msg.pose.pose.position.y))
    print ("orientacion x = " + str(msg.pose.pose.orientation.x))
    print ("orientacion y = " + str(msg.pose.pose.orientation.y))
    x_or = float(msg.pose.pose.orientation.x)
    y_or = float(msg.pose.pose.orientation.y)
    rate.sleep()

def imu_callback(msg):
    rate = rospy.Rate(0.5)
    global z_imu
    # allez = Imu()
    print ("------------------------------------------------")
    print ("veloc angular z = " + str(msg.angular_velocity.z))
    print ("veloc angular y = " + str(msg.angular_velocity.y))
    print ("aceleracion linear x = " + str(msg.linear_acceleration.x))
    print ("aceleracion linear y = " + str(msg.linear_acceleration.y))

    z_imu = float(msg.angular_velocity.z)
    rate.sleep()

def gps_callback(msg):
    global robot_gps_latitude
    global robot_gps_longitude

    global robot_gps_latitude_memory
    global robot_gps_longitude_memory


    rate = rospy.Rate(0.5)

    robot_gps_latitude = round(msg.latitude, 6)
    robot_gps_longitude = round(msg.longitude, 6)


    print ("------------------------------------------------")
    print ("latitude = " + str(robot_gps_latitude))
    print ("longitude = " + str(robot_gps_longitude))

    if robot_gps_latitude_memory == 0.0:
        robot_gps_latitude_memory = robot_gps_latitude


        degr = calculate_bearing_heading()

        print("angle to polaris: ",degr)

        move = Twist()

        # move.linear.x = 0.1 # m/s. The original value 2 is too large
        # move.angular.z= 0.5 # rad/s
        move.angular.z= degr # rad/s

        pub.publish(move)

        rate.sleep()
    else:
        error_distance_lat = abs(robot_gps_latitude_memory - robot_gps_latitude)

        print("")
        print("-----------------------------------\n")

        print("latitude in memory: ",robot_gps_latitude_memory)
        print("current latitude: ",robot_gps_latitude)
        print("ERROR:",error_distance_lat)
        if math.isclose(error_distance_lat, 0.0, abs_tol = 0.4) == True:
        # if error_distance_lat == 0:
            print("robot latitude pos is the same! \n")

        print("")
        move = Twist()

        # move.linear.x = 0.1 # m/s. The original value 2 is too large
        # move.angular.z= 0.5 # rad/s
        move.angular.z= 0.0 # rad/s

        pub.publish(move)

        rate.sleep()


def twist (msg):
    rate = rospy.Rate(0.5)
    # move = Twist()
    print ("velocidad linear x = " + str(move.linear.x))
    print ("velocidad angular z = " + str (move.angular.z))
    rate.sleep()
    #sub=rospy.Subscriber('cmd_vel', Twist, twist)

def calculate_bearing_heading():

    # θa, robot_gps_latitude
    # La, robot_gps_longitude
    # θb, polaris_latitud = 90
    # Lb, polaris_longitud = 90

    global robot_gps_latitude
    global robot_gps_longitude


    # error_distance_lat = abs(robot_gps_latitude_memory - robot_gps_latitude)
    # error_distance_lon = abs(robot_gps_longitude_memory - robot_gps_longitude)

    # if error_distance_lat == 0  error_distance_lon == 0:


    θa = robot_gps_latitude
    La = robot_gps_longitude
    θb = 90 - robot_gps_latitude
    Lb = 90 - robot_gps_longitude

    # Let ‘R’ be the radius of Earth,
    # ‘L’ be the longitude,
    # ‘θ’ be latitude,
    # ‘β‘ be Bearing.
    #  ∆L = Lb - La
    # X = cos θb * sin ∆L
    # Y = cos θa * sin θb – sin θa * cos θb * cos ∆L
    # β = atan2(X,Y)

    deltaL = Lb - La
    X = math.cos(math.radians(θb)) * math.sin(math.radians(deltaL)) #radians
    Y = math.cos(math.radians(θa)) * math.sin(math.radians(θb)) - math.sin(math.radians(θa)) * math.cos(math.radians(θb)) * math.cos(math.radians(deltaL))

    β = math.atan2(X,Y)

    degr = math.degrees(β)

    # robot_gps_latitude_memory = robot_gps_latitude
    # robot_gps_longitude_memory = robot_gps_longitude



    print("∆L: ",deltaL)
    print("X: ",X)
    print("Y: ",Y)
    print("β: ",β)
    print("degr: ",degr)





    return β



if __name__ == "__main__":


    rospy.init_node('gps_monitor') # the original name sphero might be the same as other node.
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #topic publisher that allows you to move the sphero
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback) # the original name odom might be the same as other function.
    sub_imu = rospy.Subscriber('/imu', Imu, imu_callback)
    sub_gps = rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)



    # degr = calculate_bearing_heading(39.099912, -94.581213,38.627089, -90.200203)


    rate = rospy.Rate(0.5)

    x_ground_truth = 0.0
    y_ground_truth = 0.0

    # rospy.spin()

    # # IMU heading 0 when facing east
    while not rospy.is_shutdown():
        print("waiting...")

        rate.sleep() # Instead of using rospy.spin(), we should use rate.sleep because we are in a loop

    move.angular.z= 0.0 # rad/s
    pub.publish(move)

import rospy
import tf
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('velodyne_tf')

    # TF 변환 발행을 위한 TF 브로드캐스터 초기화
    tf_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 변환 업데이트 속도 설정

    while not rospy.is_shutdown():
        try:
            # LiDAR (velodyne) 프레임과 base_link 프레임 간의 변환 정보 설정
            tf_broadcaster.sendTransform(
                (1.60, 0.0, 1.2),  # 위치 (x, y, z)
                (1.0, 0.0, 0.0, 0.0),  # 자세 (사원수 표현)
                rospy.Time.now(),  # 현재 시간 사용
                "velodyne",  # 부모 프레임 (LiDAR 프레임)
                "base_link"  # 자식 프레임 (base_link 프레임)
            )
        except Exception as e:
            rospy.logerr("Failed to publish LiDAR TF: {}".format(e))

        rate.sleep()

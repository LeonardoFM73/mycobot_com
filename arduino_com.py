import rospy
from std_msgs.msg import Int32

def menu_sender():
    rospy.init_node('gripper',anoymous=True)

    pub = rospy.Publisher('menu', Int32, queue_size=10)

    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        print("Pilihan Menu:")
        print("1. Diam")
        print("2. Kirim nilai sensor ke ROS")
        print("3. Kirim posisi servo ke ROS")

        # Minta pengguna untuk memilih menu (1, 2, atau 3)
        menu_choice = input("Masukkan pilihan (1/2/3): ")

        try:
            menu_choice = int(menu_choice)
            if menu_choice < 1 or menu_choice > 3:
                rospy.logwarn("Pilihan tidak valid. Masukkan angka 1, 2, atau 3.")
                continue

            # Buat pesan untuk dikirim ke Arduino
            msg = Int32()
            msg.data = menu_choice

            # Kirim pesan ke topik 'menu_choice'
            pub.publish(msg)

        except ValueError:
            rospy.logerr("Masukan tidak valid. Masukkan angka 1, 2, atau 3.")

        rate.sleep()

if __name__ == '__main__':
    try:
        menu_sender()
    except rospy.ROSInterruptException:
        pass
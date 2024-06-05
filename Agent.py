import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import time
import numpy as np
import pandas as pd
from keras.models import load_model
import glob
import os
import statistics

cond = False
camerax = 0
cameray = 0
cameraz = 0
class Sender(Node):
    def __init__(self):
        super().__init__('Agent')

        self.pub_menu = self.create_publisher(
            msg_type=String,
            topic='menu',
            qos_profile=10
        )

        self.pub_mycobot = self.create_publisher(
            msg_type=JointState,
            topic="koordinat",
            qos_profile=10
        )
        self.sub_mycobot = self.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self.callback_coord,
            qos_profile=10,
        )
        self.sub_servo = self.create_subscription(
            msg_type=String,
            topic="Status_collect",
            callback=self.callback_servo,
            qos_profile=10,
        )
        self.camera = self.create_subscription(
            msg_type=Point,
            topic="camera",
            callback=self.callback_camera,
            qos_profile=10,
        )


        # pub joint state
        self.joint_state_send = JointState()
        self.joint_state_send.header = Header()

        self.joint_state_send.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]

        self.joint_state_send.velocity = [0.0, ]
        self.joint_state_send.effort = []

    def send_robot(self,coordinate):
            self.joint_state_send.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_send.position = coordinate
            # self.joint_state_send.velocity = 50
            self.pub_mycobot.publish(self.joint_state_send)

    def send_command(self, command):
        msg = String()
        msg.data = str(command)
        self.pub_menu.publish(msg)
        self.get_logger().info(f'Sending command: {msg.data}')

    def callback_coord(self,msg):
        global coord_data
        coord_data = list(msg.position)
    def callback_servo(self,msg):
        global cond  # Gunakan variabel global cond
        self.status = msg.data
        # self.get_logger().info(f'Sending command: {self.status}')
        if self.status == '6':
            cond=True

    def callback_camera(self,msg):
        global camerax,cameray,cameraz,menu_choice
        if menu_choice in ['4','5','6']:
            camerax=msg.x
            cameray=msg.y
            cameraz=msg.z
            # self.get_logger().info(f"Received Point: x={msg.x}, y={msg.y}, z={msg.z}")

def detect(classifier):
    # Tentukan path ke folder yang berisi file .csv
    folder_path = 'Sensor'

    # Cari semua file .csv dalam folder tersebut
    csv_files = glob.glob(os.path.join(folder_path, '*.csv'))
    

    # Periksa apakah ada file .csv dalam folder
    if not csv_files:
        print("Tidak ada file .csv di folder ini.")
    else:
        # Temukan file .csv terbaru berdasarkan waktu modifikasi
        latest_file = max(csv_files, key=os.path.getmtime)
        print(latest_file)
    # df_test=pd.read_csv(latest_file,header=None)
    item=1
    i=0

    combined_df_final1=pd.DataFrame()

    # Read all CSV files once and store them
    dataframes = []
    df = pd.read_csv(latest_file,header=None)
    dataframes.append(df)

    while i < 128:
        i += 4
        data1 = np.zeros((1, 1500))  # Assume 1500 columns as a placeholder
        data2 = np.zeros((1, 1500))
        data3 = np.zeros((1, 1500))
        
        for idx, df in enumerate(dataframes):
            df_new1 = df.iloc[:, i].values
            df_new2 = df.iloc[:, i+1].values
            df_new3 = df.iloc[:, i+2].values
            data1[idx, :len(df_new1)] = df_new1
            data2[idx, :len(df_new2)] = df_new2
            data3[idx, :len(df_new3)] = df_new3

        combined_df1 = pd.DataFrame(data1).fillna(0)
        combined_df2 = pd.DataFrame(data2).fillna(0)
        combined_df3 = pd.DataFrame(data3).fillna(0)

        # Ensure there are 1501 columns by adding additional columns if necessary
        num_cols = 1500
        combined_df1 = combined_df1.reindex(columns=range(num_cols), fill_value=0)
        combined_df2 = combined_df2.reindex(columns=range(num_cols), fill_value=0)
        combined_df3 = combined_df3.reindex(columns=range(num_cols), fill_value=0)

        combined_df_final1[f'Nilai X{item}'] = combined_df1.values.tolist()
        combined_df_final1[f'Nilai Y{item}'] = combined_df2.values.tolist()
        combined_df_final1[f'Nilai Z{item}'] = combined_df3.values.tolist()
        item += 1
        
    X1=combined_df_final1[f'Nilai X1'].tolist()
    Y1=combined_df_final1[f'Nilai Y1'].tolist()
    Z1=combined_df_final1[f'Nilai Z1'].tolist()

    X2=combined_df_final1[f'Nilai X2'].tolist()
    Y2=combined_df_final1[f'Nilai Y2'].tolist()
    Z2=combined_df_final1[f'Nilai Z2'].tolist()

    X3=combined_df_final1[f'Nilai X3'].tolist()
    Y3=combined_df_final1[f'Nilai Y3'].tolist()
    Z3=combined_df_final1[f'Nilai Z3'].tolist()

    X4=combined_df_final1[f'Nilai X4'].tolist()
    Y4=combined_df_final1[f'Nilai Y4'].tolist()
    Z4=combined_df_final1[f'Nilai Z4'].tolist()

    X5=combined_df_final1[f'Nilai X5'].tolist()
    Y5=combined_df_final1[f'Nilai Y5'].tolist()
    Z5=combined_df_final1[f'Nilai Z5'].tolist()

    X6=combined_df_final1[f'Nilai X6'].tolist()
    Y6=combined_df_final1[f'Nilai Y6'].tolist()
    Z6=combined_df_final1[f'Nilai Z6'].tolist()

    X7=combined_df_final1[f'Nilai X7'].tolist()
    Y7=combined_df_final1[f'Nilai Y7'].tolist()
    Z7=combined_df_final1[f'Nilai Z7'].tolist()

    X8=combined_df_final1[f'Nilai X8'].tolist()
    Y8=combined_df_final1[f'Nilai Y8'].tolist()
    Z8=combined_df_final1[f'Nilai Z8'].tolist()

    X9=combined_df_final1[f'Nilai X9'].tolist()
    Y9=combined_df_final1[f'Nilai Y9'].tolist()
    Z9=combined_df_final1[f'Nilai Z9'].tolist()

    X10=combined_df_final1[f'Nilai X10'].tolist()
    Y10=combined_df_final1[f'Nilai Y10'].tolist()
    Z10=combined_df_final1[f'Nilai Z10'].tolist()

    X11=combined_df_final1[f'Nilai X11'].tolist()
    Y11=combined_df_final1[f'Nilai Y11'].tolist()
    Z11=combined_df_final1[f'Nilai Z11'].tolist()

    X12=combined_df_final1[f'Nilai X12'].tolist()
    Y12=combined_df_final1[f'Nilai Y12'].tolist()
    Z12=combined_df_final1[f'Nilai Z12'].tolist()

    X13=combined_df_final1[f'Nilai X13'].tolist()
    Y13=combined_df_final1[f'Nilai Y13'].tolist()
    Z13=combined_df_final1[f'Nilai Z13'].tolist()

    X14=combined_df_final1[f'Nilai X14'].tolist()
    Y14=combined_df_final1[f'Nilai Y14'].tolist()
    Z14=combined_df_final1[f'Nilai Z14'].tolist()

    X15=combined_df_final1[f'Nilai X15'].tolist()
    Y15=combined_df_final1[f'Nilai Y15'].tolist()
    Z15=combined_df_final1[f'Nilai Z15'].tolist()

    X16=combined_df_final1[f'Nilai X16'].tolist()
    Y16=combined_df_final1[f'Nilai Y16'].tolist()
    Z16=combined_df_final1[f'Nilai Z16'].tolist()

    X17=combined_df_final1[f'Nilai X17'].tolist()
    Y17=combined_df_final1[f'Nilai Y17'].tolist()
    Z17=combined_df_final1[f'Nilai Z17'].tolist()

    X18=combined_df_final1[f'Nilai X18'].tolist()
    Y18=combined_df_final1[f'Nilai Y18'].tolist()
    Z18=combined_df_final1[f'Nilai Z18'].tolist()

    X19=combined_df_final1[f'Nilai X19'].tolist()
    Y19=combined_df_final1[f'Nilai Y19'].tolist()
    Z19=combined_df_final1[f'Nilai Z19'].tolist()

    X20=combined_df_final1[f'Nilai X20'].tolist()
    Y20=combined_df_final1[f'Nilai Y20'].tolist()
    Z20=combined_df_final1[f'Nilai Z20'].tolist()

    X21=combined_df_final1[f'Nilai X21'].tolist()
    Y21=combined_df_final1[f'Nilai Y21'].tolist()
    Z21=combined_df_final1[f'Nilai Z21'].tolist()

    X22=combined_df_final1[f'Nilai X22'].tolist()
    Y22=combined_df_final1[f'Nilai Y22'].tolist()
    Z22=combined_df_final1[f'Nilai Z22'].tolist()

    X23=combined_df_final1[f'Nilai X23'].tolist()
    Y23=combined_df_final1[f'Nilai Y23'].tolist()
    Z23=combined_df_final1[f'Nilai Z23'].tolist()

    X24=combined_df_final1[f'Nilai X24'].tolist()
    Y24=combined_df_final1[f'Nilai Y24'].tolist()
    Z24=combined_df_final1[f'Nilai Z24'].tolist()

    X25=combined_df_final1[f'Nilai X25'].tolist()
    Y25=combined_df_final1[f'Nilai Y25'].tolist()
    Z25=combined_df_final1[f'Nilai Z25'].tolist()

    X26=combined_df_final1[f'Nilai X26'].tolist()
    Y26=combined_df_final1[f'Nilai Y26'].tolist()
    Z26=combined_df_final1[f'Nilai Z26'].tolist()

    X27=combined_df_final1[f'Nilai X27'].tolist()
    Y27=combined_df_final1[f'Nilai Y27'].tolist()
    Z27=combined_df_final1[f'Nilai Z27'].tolist()

    X28=combined_df_final1[f'Nilai X28'].tolist()
    Y28=combined_df_final1[f'Nilai Y28'].tolist()
    Z28=combined_df_final1[f'Nilai Z28'].tolist()

    X29=combined_df_final1[f'Nilai X29'].tolist()
    Y29=combined_df_final1[f'Nilai Y29'].tolist()
    Z29=combined_df_final1[f'Nilai Z29'].tolist()

    X30=combined_df_final1[f'Nilai X30'].tolist()
    Y30=combined_df_final1[f'Nilai Y30'].tolist()
    Z30=combined_df_final1[f'Nilai Z30'].tolist()

    X31=combined_df_final1[f'Nilai X31'].tolist()
    Y31=combined_df_final1[f'Nilai Y31'].tolist()
    Z31=combined_df_final1[f'Nilai Z31'].tolist()

    X32=combined_df_final1[f'Nilai X32'].tolist()
    Y32=combined_df_final1[f'Nilai Y32'].tolist()
    Z32=combined_df_final1[f'Nilai Z32'].tolist()

    X_train3=np.concatenate((X1, Y1, Z1, X2, Y2, Z2,X3, Y3,Z3,X4,Y4,Z4,X5,Y5,Z5,X6,Y6,Z6,X7,Y7,Z7,X8,Y8,Z8,X9,Y9,Z9,X10,Y10,Z10,X11,Y11,Z11,X12,Y12,Z12,X13,Y13,Z13,X14,Y14,Z14,X15,Y15,Z15,X16,Y16,Z16, X17, Y17, Z17, X18, Y18, Z18,X19, Y19,Z19,X20,Y20,Z20,X21,Y21,Z21,X22,Y22,Z22,X23,Y23,Z23,X24,Y24,Z24,X25,Y25,Z25,X26,Y26,Z26,X27,Y27,Z27,X28,Y28,Z28,X29,Y29,Z29,X30,Y30,Z30,X31,Y31,Z31,X32,Y32,Z32), axis=1)

    labels = ['Buah Segar', 'Buah Rusak', 'Buah Busuk']
    jenis = classifier.predict(X_train3)
    max_index = np.argmax(jenis)
    predicted_label = labels[max_index]

    print("Predicted label:", predicted_label)

    return jenis, max_index

def detect_cnn(classifier):
    # Tentukan path ke folder yang berisi file .csv
    folder_path = 'Sensor'

    # Cari semua file .csv dalam folder tersebut
    csv_files = glob.glob(os.path.join(folder_path, '*.csv'))
    

    # Periksa apakah ada file .csv dalam folder
    if not csv_files:
        print("Tidak ada file .csv di folder ini.")
    else:
        # Temukan file .csv terbaru berdasarkan waktu modifikasi
        latest_file = max(csv_files, key=os.path.getmtime)
        print(latest_file)
    # df_test=pd.read_csv(latest_file,header=None)
    item=1
    i=0

    combined_df_final1=pd.DataFrame()

    # Read all CSV files once and store them
    dataframes = []
    df = pd.read_csv(latest_file,header=None)
    dataframes.append(df)

    while i < 128:
        i += 4
        data1 = np.zeros((1, 1500))  # Assume 1500 columns as a placeholder
        data2 = np.zeros((1, 1500))
        data3 = np.zeros((1, 1500))
        
        for idx, df in enumerate(dataframes):
            df_new1 = df.iloc[:, i].values
            df_new2 = df.iloc[:, i+1].values
            df_new3 = df.iloc[:, i+2].values
            data1[idx, :len(df_new1)] = df_new1
            data2[idx, :len(df_new2)] = df_new2
            data3[idx, :len(df_new3)] = df_new3

        combined_df1 = pd.DataFrame(data1).fillna(0)
        combined_df2 = pd.DataFrame(data2).fillna(0)
        combined_df3 = pd.DataFrame(data3).fillna(0)

        # Ensure there are 1501 columns by adding additional columns if necessary
        num_cols = 1500
        combined_df1 = combined_df1.reindex(columns=range(num_cols), fill_value=0)
        combined_df2 = combined_df2.reindex(columns=range(num_cols), fill_value=0)
        combined_df3 = combined_df3.reindex(columns=range(num_cols), fill_value=0)

        combined_df_final1[f'Nilai X{item}'] = combined_df1.values.tolist()
        combined_df_final1[f'Nilai Y{item}'] = combined_df2.values.tolist()
        combined_df_final1[f'Nilai Z{item}'] = combined_df3.values.tolist()
        item += 1
        
    # Definisikan parameter input
    num_sensors = 32
    data_length = 1500
    input_shape = (data_length, 3)  # Karena setiap sensor memiliki x, y, z

    # Inisialisasi array untuk menyimpan data yang sudah diproses
    num_samples = len(combined_df_final1)
    processed_data1 = np.zeros((num_samples, num_sensors, data_length, 3))

    # Proses setiap kolom yang mengandung daftar nilai
    for sensor_idx in range(num_sensors):
        for axis_idx, axis in enumerate(['X', 'Y', 'Z']):
            col_name = f'Nilai {axis}{sensor_idx + 1}'
            processed_data1[:, sensor_idx, :, axis_idx] = combined_df_final1[col_name].apply(lambda x: eval(x) if isinstance(x, str) else x).tolist()
    
    # Reshape data baru sesuai dengan input shape yang dibutuhkan oleh model
    new_data = processed_data1.reshape(-1, data_length, 3)
    # Melakukan prediksi
    predictions = classifier.predict(new_data)
    # Mengambil kelas dengan probabilitas tertinggi
    predicted_classes = np.argmax(predictions, axis=1)
    modus = statistics.mode(predicted_classes)
    labels = ['Buah Segar', 'Buah Rusak', 'Buah Busuk']
    predicted_label = labels[modus]
    print("Predicted label:", predicted_label)

    return predictions, modus

def main(args=None):
    global cond,camerax,cameray,cameraz,menu_choice  # Gunakan variabel global con
    print("Processing Model...")
    # classifier = joblib.load('my_model.joblib')
    model = load_model('my_model.h5')
    model1 = load_model('modelCNN.h5')
    os.system("clear")

    rclpy.init(args=args)
    sender = Sender()

    try:
        while rclpy.ok():
            camerax = 0
            cameray = 0
            cameraz = 0
            start = [float(-42.2),float(-163.2),float(106.3),float(-55.10),float(-3.07),float(-161.00)] #y+15 z+20
            sender.send_robot(start)
            cond = False
            print("Pilihan Menu:")
            print("1. Buka")
            print("2. Tutup")
            print("3. Collect Data")
            print("4. Collect Data(Camera)")
            print("5. Detect(ANN)")
            print("6. Detect(CNN)")
            

            menu_choice = input("Masukkan pilihan (1/2/3/4/5/6): ")

            if menu_choice in ['1', '2']:
                sender.send_command(menu_choice)
            elif menu_choice in ['3']:
                # res = [float(-68.4),float(-219.8),float(300.8),float(-85.69),float(1.39),float(179.84)]
                # res = [float(49.5),float(-185.3),float(280.5),float(-85.10),float(3),float(179.64)]
                # res = [float(-48.0),float(-150.0),float(150.5),float(-80.00),float(0),float(179.64)]
                # sender.send_robot(res)
                # time.sleep(3)
                res = [float(-48.0),float(-150.0),float(221.7),float(-80.00),float(0),float(-161.00)] 
                sender.send_robot(res)
                time.sleep(2)
                sender.send_command(menu_choice)
                while not cond:
                    rclpy.spin_once(sender)
                    print(cond)
                    # time.sleep(5) # input dari sensor
                # res1 = [float(37.1),float(-161.3),float(224.0),float(-82.62),float(0.67),float(-142.35)]
                # sender.send_robot(res1)
                time.sleep(2)
                sender.send_command('1')
                # sender.send_robot(res)
                time.sleep(3)
                sender.send_command('10')
                # cond = False
            elif menu_choice in ['4']:
                camerax = 0
                cameray = 0
                cameraz = 0
                
                rclpy.spin_once(sender)

                while camerax == 0:
                    rclpy.spin_once(sender)
                    print(float(camerax),float(cameray),float(cameraz))
                
                # Buat vektor posisi awal
                posisi = np.array([camerax, cameray, cameraz])

                # Matriks rotasi -90 derajat sekitar sumbu x
                theta_x = np.radians(-90)
                rotasi_x = np.array([
                    [1, 0, 0],
                    [0, np.cos(theta_x), -np.sin(theta_x)],
                    [0, np.sin(theta_x), np.cos(theta_x)]
                ])

                # Terapkan rotasi pada sumbu x
                posisi_rotasi_x = rotasi_x.dot(posisi)

                # Matriks rotasi -180 derajat sekitar sumbu z
                theta_z = np.radians(-180)
                rotasi_z = np.array([
                    [np.cos(theta_z), -np.sin(theta_z), 0],
                    [np.sin(theta_z), np.cos(theta_z), 0],
                    [0, 0, 1]
                ])

                # Terapkan rotasi pada sumbu z
                posisi_rotasi_z = rotasi_z.dot(posisi_rotasi_x)
                posisi_akhir = posisi_rotasi_z + np.array([40, 201 - posisi_rotasi_z[0]*0.05, 288 + posisi_rotasi_z[0]*0.095]) # 30, 200 , 280
                posisi_akhir_b = [round(elem, 1) for elem in posisi_akhir ]

                res = [float(posisi_akhir_b[0]),float(posisi_akhir_b[1]),float(posisi_akhir_b[2]-30),float(-80.10),float(0),float(-161.00)] #y+15 z+20
                sender.send_robot(res)
                # # print(res)
                time.sleep(4)
                res = [float(posisi_akhir_b[0]),float(posisi_akhir_b[1]),float(posisi_akhir_b[2]),float(-80.10),float(0),float(-161.00)] #y+15 z+20
                print(res)
                sender.send_robot(res)
                time.sleep(5)
                sender.send_command('3')
                while not cond:
                    rclpy.spin_once(sender)
                    print(cond)
                # res1 = [float(37.1),float(-161.3),float(224.0),float(-80.62),float(0.67),float(-142.35)]
                # sender.send_robot(res1)
                time.sleep(2)

                sender.send_command('1')
                # sender.send_robot(res)
                time.sleep(1)
                sender.send_command('10')

            elif menu_choice in ['5']:
                camerax = 0
                cameray = 0
                cameraz = 0
                
                rclpy.spin_once(sender)

                while camerax == 0:
                    rclpy.spin_once(sender)
                    print(float(camerax),float(cameray),float(cameraz))
                
                # Buat vektor posisi awal
                posisi = np.array([camerax, cameray, cameraz])

                # Matriks rotasi -90 derajat sekitar sumbu x
                theta_x = np.radians(-90)
                rotasi_x = np.array([
                    [1, 0, 0],
                    [0, np.cos(theta_x), -np.sin(theta_x)],
                    [0, np.sin(theta_x), np.cos(theta_x)]
                ])

                # Terapkan rotasi pada sumbu x
                posisi_rotasi_x = rotasi_x.dot(posisi)

                # Matriks rotasi -180 derajat sekitar sumbu z
                theta_z = np.radians(-180)
                rotasi_z = np.array([
                    [np.cos(theta_z), -np.sin(theta_z), 0],
                    [np.sin(theta_z), np.cos(theta_z), 0],
                    [0, 0, 1]
                ])

                # Terapkan rotasi pada sumbu z
                posisi_rotasi_z = rotasi_z.dot(posisi_rotasi_x)
                posisi_akhir = posisi_rotasi_z + np.array([float(40), float(212 - posisi_rotasi_z[0]*0.0675), float(292 + posisi_rotasi_z[0]*0.125)]) # 30, 200 , 280
                posisi_akhir_b = [round(elem, 1) for elem in posisi_akhir ]

                res = [float(posisi_akhir_b[0]),float(posisi_akhir_b[1]),float(posisi_akhir_b[2]-30),float(-80.10),float(0),float(-161.00)] #y+15 z+20
                sender.send_robot(res)
                # # print(res)
                time.sleep(4)
                res = [float(posisi_akhir_b[0]),float(posisi_akhir_b[1]),float(posisi_akhir_b[2]),float(-80.10),float(0),float(-161.00)] #y+15 z+20
                print(res)
                sender.send_robot(res)
                time.sleep(5)
                sender.send_command('3')
                while not cond:
                    rclpy.spin_once(sender)
                    print(cond)
                # res1 = [float(37.1),float(-161.3),float(224.0),float(-80.62),float(0.67),float(-142.35)]
                # sender.send_robot(res1)
                time.sleep(2)
                result, jenis = detect(model)
                print(result)
                if jenis == 0:
                    pot = [float(195.3),float(-53.8),float(272.9),float(-166.6),float(1.19),float(-85.71)] #y+15 z+20
                    print('Buah Segar')
                elif jenis == 1:
                    pot = [float(174.0),float(-172.9),float(272.9),float(-148.21),float(-3.32),float(-119.57)] #y+15 z+20
                    print('Buah Rusak')
                elif jenis == 2:
                    pot = [float(174.0),float(-172.9),float(272.9),float(-148.21),float(-3.32),float(-119.57)] #y+15 z+20
                    print('Buah Busuk')
                # pot = [float(174.0),float(-172.9),float(242.9),float(-148.21),float(-3.32),float(-119.57)] #y+15 z+20
                # sender.send_robot(pot1)
                sender.send_robot(pot)
                time.sleep(2)

                sender.send_command('1')
                # sender.send_robot(res)
                time.sleep(1)
                sender.send_command('10')

            elif menu_choice in ['6']:
                camerax = 0
                cameray = 0
                cameraz = 0
                
                rclpy.spin_once(sender)

                while camerax == 0:
                    rclpy.spin_once(sender)
                    print(float(camerax),float(cameray),float(cameraz))
                
                # Buat vektor posisi awal
                posisi = np.array([camerax, cameray, cameraz])

                # Matriks rotasi -90 derajat sekitar sumbu x
                theta_x = np.radians(-90)
                rotasi_x = np.array([
                    [1, 0, 0],
                    [0, np.cos(theta_x), -np.sin(theta_x)],
                    [0, np.sin(theta_x), np.cos(theta_x)]
                ])

                # Terapkan rotasi pada sumbu x
                posisi_rotasi_x = rotasi_x.dot(posisi)

                # Matriks rotasi -180 derajat sekitar sumbu z
                theta_z = np.radians(-180)
                rotasi_z = np.array([
                    [np.cos(theta_z), -np.sin(theta_z), 0],
                    [np.sin(theta_z), np.cos(theta_z), 0],
                    [0, 0, 1]
                ])

                # Terapkan rotasi pada sumbu z
                posisi_rotasi_z = rotasi_z.dot(posisi_rotasi_x)
                posisi_akhir = posisi_rotasi_z + np.array([float(40), float(212 - posisi_rotasi_z[0]*0.0675), float(270 + posisi_rotasi_z[0]*0.125)]) # 30, 200 , 280
                posisi_akhir_b = [round(elem, 1) for elem in posisi_akhir ]

                res = [float(posisi_akhir_b[0]),float(posisi_akhir_b[1]),float(posisi_akhir_b[2]-30),float(-80.10),float(0),float(-161.00)] #y+15 z+20
                sender.send_robot(res)
                # # print(res)
                time.sleep(4)
                res = [float(posisi_akhir_b[0]),float(posisi_akhir_b[1]),float(posisi_akhir_b[2]),float(-80.10),float(0),float(-161.00)] #y+15 z+20
                print(res)
                sender.send_robot(res)
                time.sleep(5)
                sender.send_command('3')
                while not cond:
                    rclpy.spin_once(sender)
                    print(cond)
                # res1 = [float(37.1),float(-161.3),float(224.0),float(-80.62),float(0.67),float(-142.35)]
                # sender.send_robot(res1)
                time.sleep(2)
                result, jenis = detect_cnn(model1)
                print(result)
                if jenis == 0:
                    pot = [float(195.3),float(-53.8),float(272.9),float(-166.6),float(1.19),float(-85.71)] #y+15 z+20
                    print('Buah Segar')
                elif jenis == 1:
                    pot = [float(174.0),float(-172.9),float(272.9),float(-148.21),float(-3.32),float(-119.57)] #y+15 z+20
                    print('Buah Rusak')
                elif jenis == 2:
                    pot = [float(174.0),float(-172.9),float(272.9),float(-148.21),float(-3.32),float(-119.57)] #y+15 z+20
                    print('Buah Busuk')
                # pot = [float(174.0),float(-172.9),float(242.9),float(-148.21),float(-3.32),float(-119.57)] #y+15 z+20
                # sender.send_robot(pot1)
                sender.send_robot(pot)
                time.sleep(2)

                sender.send_command('1')
                # sender.send_robot(res)
                time.sleep(1)
                sender.send_command('10')
            else:
                print("Pilihan tidak valid. Masukkan angka 1, 2, atau 3.")

    except KeyboardInterrupt:
        pass

    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

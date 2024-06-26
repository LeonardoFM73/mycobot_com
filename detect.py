import pandas as pd
import os
import timeit
import numpy as np
import joblib

class Detect:
    def __init__(self):
        print("Processing Model...")
        joblib.dump(self.classifier,'my_model.joblib')
    def detect(self):
        df_test=pd.read_csv("/content/drive/MyDrive/Dataset/Data/Campur/apple_royal_GU6_16_16x3_11102023_94742.csv",header=None)
        item=1
        i=0

        combined_df_final1=pd.DataFrame()

        while (i<128):
            i+=4
            dataframes1 =[]
            dataframes2 =[]
            dataframes3 =[]
            df_new1=df_test.iloc[:,i].values
            df_new2=df_test.iloc[:,i+1].values
            df_new3=df_test.iloc[:,i+2].values
            d1 = pd.DataFrame(df_new1.reshape(1,-1))
            d2 = pd.DataFrame(df_new2.reshape(1,-1))
            d3 = pd.DataFrame(df_new3.reshape(1,-1))
            dataframes1.append(d1)
            dataframes2.append(d2)
            dataframes3.append(d3)
            combined_df1 = pd.concat(dataframes1, ignore_index=True).fillna(0)
            combined_df2 = pd.concat(dataframes2, ignore_index=True).fillna(0)
            combined_df3 = pd.concat(dataframes3, ignore_index=True).fillna(0)

            nomor_terakhir = int(combined_df1.columns[-1])
            jumlah_kolom_baru = 717

            for x in range(1, jumlah_kolom_baru-nomor_terakhir):
                nama_kolom_baru = f'{nomor_terakhir + x}'
                combined_df1[nama_kolom_baru] = 0
                combined_df2[nama_kolom_baru] = 0
                combined_df3[nama_kolom_baru] = 0

            combined_df_final1[f'Nilai X{item}']=combined_df1.iloc[:, :-1].values.tolist()
            combined_df_final1[f'Nilai Y{item}']=combined_df2.iloc[:, :-1].values.tolist()
            combined_df_final1[f'Nilai Z{item}']=combined_df3.iloc[:, :-1].values.tolist()
            item+=1

        
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

        labels = ['tomato', 'apple', 'pear', 'orange']
        jenis = self.classifier.predict(X_train3)
        max_index = np.argmax(jenis)
        predicted_label = labels[max_index]

        print("Predicted label:", predicted_label)
from machine import UART
import sensor,  image, lcd,  time
from fpioa_manager import fm
from Maix import GPIO


fm.register(35, fm.fpioa.UART1_TX, force=True)
fm.register(34, fm.fpioa.UART1_RX, force=True)

fm.register(18, fm.fpioa.GPIO1)
ButonA=GPIO(GPIO.GPIO1, GPIO.IN, GPIO.PULL_UP)
fm.register(19, fm.fpioa.GPIO2)
ButonB=GPIO(GPIO.GPIO2, GPIO.IN, GPIO.PULL_UP)

uart = UART(UART.UART1, 115200,8,0,0, timeout=1000, read_buf_len=4096)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((320,240))
sensor.set_brightness(2)
sensor.set_vflip(1)

sensor.run(1)
#blue_threshold   = [(0, 100, -127, 125, -78, -37)]
blue_threshold = [(18, 100, -128, 120, -128, -45)]
yellow_threshold = [(0, 100, -34, 45, 46, 127)]
Y_th = yellow_threshold[0]
B_th = blue_threshold[0]
GAIN = 60.0
WHITE_GAIN = (83.0, 64.0, 121.0)

myroi = (0,90,320,110)
senter = (150,0,20,240)
senter_flag = 0
A = 1  #0なら青　1なら黄色
B = 999

clock = time.clock()


sensor.set_auto_gain(1)
sensor.set_auto_whitebal(1)
a = 0
t = time.ticks_ms()

while time.ticks_ms() - t < 1000:
    a += 1

print("gain = ",end="")
print(sensor.get_gain_db())
print(" WHITE_BAL = ",end="")
print(sensor.get_rgb_gain_db())

sensor.set_auto_gain(False,gain_gb = GAIN,gain_db_ceiling = GAIN)
sensor.set_auto_whitebal(False,WHITE_GAIN)

while True:
    senter_flag = 0
    clock.tick()
    img = sensor.snapshot()
    send_list = [38,0,0,0,0,0,0,37]

    if img:     #画像がなかったらエラー起こるからね、しょうがないね
        try:    #よくfind_blobsで例外起こるからｔｒｙ文に入れてる
            blobs_blue = img.find_blobs(blue_threshold,roi = myroi,pixels_threshold = 100)
            blobs_yellow = img.find_blobs(yellow_threshold,roi = myroi,pixels_threshold = 100)
            if img.find_blobs(yellow_threshold,roi = senter) or img.find_blobs(blue_threshold,roi = senter):
                senter_flag = 1     #画像の中心にブロックがあったらシュートできるよ

        except AttributeError as E: #特に意味ない
            print(E)


        if blobs_blue:      #こっから青色判定ゾーン
            max_area_b = 0  #ブロックの面積の最大値初期化
            target_b=blobs_blue[0]  #変数の初期化
            for b in blobs_blue:    #for文で回す
                if b.area() > max_area_b:   #面積を比較してもっともデカかったら記録
                    max_area_b = b.area()
                    target_b = b

            tmp=img.draw_rectangle(target_b[0:4])   #ブロックを枠でかこう
            width_b = target_b[2] / 8   #敵避け用にブロックの幅を8で割る
            height_b = target_b[3] / 20 #敵避け用にブロックの高さを8で割る
            H = 0       #ブロックごとの一番下にある青色のピクセルのある高さを記録
            H_old = 0   #見てるブロックの一つ左の高さ
            Flag = 0    #敵感知した?
            robot_range=[0,8]
            x_b=target_b[5] #ブロックの中心を座標として記録

            for i in range(0,8):    #こっから敵感知(ここは横向きに分けてる)
                X = target_b[0]+width_b*i   #ブロック分けしたx座標
                for j in range(1,20):       #高さでfor文回してる
                    Y=target_b[1]+target_b[3]-height_b*j    #高さの変数
                    pixel=img.get_pixel(int(X),int(Y))      #色を入手
                    pixel=image.rgb_to_lab(pixel)           #LABに色変換

                    if B_th[4]<pixel[2] and pixel[2]<B_th[5]:   #Bの閾値におさまってるか判定
                        tmp=img.draw_cross(int(X),int(Y),color=(0,0,200))
                        H = j           #一番したにある高さを記録
                        if 2<H - H_old: #隣り合ったピクセルの高さの差を見てるよ ある程度差があったら敵と検知
                            Flag=1
                            robot_range[0]=i
                        if H - H_old<-2:
                            Flag=1
                            robot_range[1]=i

                        H_old=H
                        break


            if Flag!=0:
                if 5<abs(robot_range[0]-robot_range[1]):    #あんまり敵の幅が大きい時はおかしいから無視する
                    Flag=0

            range_s=int(target_b[0]+width_b*robot_range[0]) #敵がいるx座標の始まり
            range_f=int(target_b[0]+width_b*robot_range[1]) #敵がいるx座標の終わり

            if Flag!=0:     #敵がいたと検知されたら
                tmp=img.draw_line(range_s,target_b[1],range_f,target_b[1],color=(0,0,200),thickness=10)
                if 8-robot_range[1]<robot_range[0]:     #なんかいろいろ都合のいいようにするやつ(何かいてるかわからん)
                    x_b=int(target_b[0]+width_b*robot_range[0]/2)
                else:
                    x_b=int(target_b[0]+width_b*(8+robot_range[1])/2)

                if (100<range_s or range_f<140)or(range_s<100 and 140<range_f):
                    senter_flag=0


            tmp=img.draw_cross(int(x_b),int(target_b[6]),color=(200,0,0),size=5)
            x_b /= 2    #1バイトにおさまるように2で割ってる

            send_list[1] = int(x_b)
            send_list[2] = int(target_b[3])
            send_list[3] = int(senter_flag)

        if blobs_yellow:    #青色と同じなので割愛
            target_w=blobs_yellow[0]
            max_area_w = 0
            for b in blobs_yellow:
                if b.area() > max_area_w:
                    max_area_w = b.area()
                    target_w = b

            tmp=img.draw_rectangle(target_w[0:4])
            width_y = target_w[2] / 8
            height_y = target_w[3] / 20
            H = 0
            H_old = 0
            Flag = 0
            robot_range=[0,8]
            x_y=target_w[5]
            for i in range(1,8):
                X = target_w[0]+width_y*i
                for j in range(1,20):
                    Y=target_w[1]+target_w[3]-height_y*j
                    pixel=img.get_pixel(int(X),int(Y))
                    pixel=image.rgb_to_lab(pixel)

                    if Y_th[4]<pixel[2] and pixel[2]<Y_th[5]:
                        tmp=img.draw_cross(int(X),int(Y),color=(0,0,200))
                        H = j
                        if i == 1:
                            H_old = H

                        if 1<H - H_old:
                            Flag+=1
                            robot_range[0]=i
                        if H - H_old<-1:
                            Flag+=2
                            robot_range[1]=i-1

                        H_old=H
                        break

            if Flag!=0:
                if 5<abs(robot_range[0]-robot_range[1]):
                    Flag=0

            range_s=int(target_w[0]+width_y*robot_range[0])
            range_f=int(target_w[0]+width_y*robot_range[1])

            if Flag!=0:
                tmp=img.draw_line(range_s,target_w[1],range_f,target_w[1],color=(0,0,200),thickness=10)
                if 8-robot_range[1]<robot_range[0]:
                    x_y=int(target_w[0]+width_y*robot_range[0]/2)
                else:
                    x_y=int(target_w[0]+width_y*(8+robot_range[1])/2)

                if (100<range_s or range_f<140)or(range_s<100 and 140<range_f):
                    senter_flag=0

            tmp=img.draw_cross(int(x_y),int(target_w[6]),color=(200,0,0),size=5)
            x_y /= 2
            send_list[4] = int(x_y)
            send_list[5] = int(target_w[3])
            send_list[6] = int(senter_flag)

    send = bytearray(send_list) #int型の変数たちをバイト型に変換
    uart.write(send)            #バイト型変数をteensyに送っちゃう

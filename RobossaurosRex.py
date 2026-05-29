from hub import light_matrix, port, motion_sensor
import color_sensor
import distance_sensor
import motor_pair
import color
import runloop

#constantes e coisas a serem declaradas
sensorD = port.D
sensorE = port.C
motor_pair.pair(motor_pair.PAIR_1,port.F,port.E)
kP = 15
areaDeResgate = False
motion_sensor.set_yaw_face(motion_sensor.TOP)

async def main():
    while not areaDeResgate:
        if color_sensor.color(sensorD)== color.RED or color_sensor.color(sensorE)== color.RED:
            motor_pair.stop(motor_pair.PAIR_1)
            break
        elif distance_sensor.distance(port.A) < 50 and distance_sensor.distance(port.A) > 0:
            motor_pair.move_tank(motor_pair.PAIR_1,-100,-100)
            await runloop.until(lambda: distance_sensor.distance(port.A)>=70)
            await darAVolta()
            seguirLinha()
            continue
        elif color_sensor.color(sensorE)== color.GREEN or color_sensor.color(sensorD)== color.GREEN:
            light_matrix.show_image(light_matrix.IMAGE_TARGET)
            motor_pair.move_tank(motor_pair.PAIR_1,100,100)
            await runloop.sleep_ms(100)
            #Se os dois forem verdes, dá meia volta
            if color_sensor.color(sensorD)== color.GREEN and color_sensor.color(sensorE)== color.GREEN:
                motion_sensor.reset_yaw(0)
                motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
                await runloop.until(lambda: verSeVirou(170))
                motor_pair.move_tank(motor_pair.PAIR_1,-200,200)
                await runloop.until(lambda: color_sensor.reflection(sensorD)==color_sensor.reflection(sensorE))
                seguirLinha()
                continue
            #Se só o direito for verde, vira à direita
            elif color_sensor.color(sensorD)== color.GREEN and color_sensor.color(sensorE)!= color.GREEN:
                await motor_pair.move_tank_for_time(motor_pair.PAIR_1,300,300,400)
                motion_sensor.reset_yaw(0)
                motor_pair.move_tank(motor_pair.PAIR_1,-120,300)
                await runloop.until(lambda: verSeVirou(-85))
                motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
                await runloop.until(lambda: color_sensor.reflection(sensorD)==color_sensor.reflection(sensorE))
                seguirLinha()
                continue
            #Se só o esquerdo for verde, vira à esquerda
            elif color_sensor.color(sensorE)== color.GREEN and color_sensor.color(sensorD)!= color.GREEN:
                await motor_pair.move_tank_for_time(motor_pair.PAIR_1,300,300,400)
                motion_sensor.reset_yaw(0)
                motor_pair.move_tank(motor_pair.PAIR_1,300,-120)
                await runloop.until(lambda: verSeVirou(85))
                motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
                await runloop.until(lambda: color_sensor.reflection(sensorD)==color_sensor.reflection(sensorE))
                seguirLinha()
                continue
        elif (color_sensor.reflection(sensorE)< 25 or color_sensor.reflection(sensorD)< 25) and abs(motion_sensor.tilt_angles()[1])<100:
            await varredura()
            continue
        elif color_sensor.reflection(sensorE) >= 30 or color_sensor.reflection(sensorD) >= 30:
            seguirLinha()
            await runloop.sleep_ms(10)
        else:
            seguirLinha()
            await runloop.sleep_ms(20)

def seguirLinha():
    #Atribuição de potência com base na diferença de reflexão entre os sensores
    light_matrix.show_image(light_matrix.IMAGE_ARROW_N)
    erro = (color_sensor.reflection(sensorE)-color_sensor.reflection(sensorD))*kP
    powerD = 300 + erro
    powerE = 300 - erro
    motor_pair.move_tank(motor_pair.PAIR_1,powerE,powerD)


async def varredura():
    virouD = False
    virouE = False
    esqRefl = color_sensor.reflection(sensorE)
    dirRefl = color_sensor.reflection(sensorD)
    light_matrix.show_image(light_matrix.IMAGE_CHESSBOARD)
    #Ver se é uma intersecção, se sim passar direto
    motor_pair.move_tank(motor_pair.PAIR_1,300,300)
    await runloop.sleep_ms(400)
    motor_pair.stop(motor_pair.PAIR_1)
    if color_sensor.color(sensorE)== color.GREEN or color_sensor.color(sensorD)== color.GREEN:
        motor_pair.move_tank(motor_pair.PAIR_1,300,300)
        await runloop.sleep_ms(400)
        seguirLinha()
        return
    if esqRefl <= 40:
            #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
        await runloop.until(lambda: verSeVirou(90) or color_sensor.reflection(sensorD)<= 40)
        motor_pair.stop(motor_pair.PAIR_1)
        if color_sensor.reflection(sensorD) <= 40:
            motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
            await runloop.until(lambda: color_sensor.color(sensorD)==color_sensor.color(sensorE))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(500)
            return

        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -300, 300)
        await runloop.until(lambda: verSeVirou(-179) or color_sensor.reflection(sensorE)<= 30)
        motor_pair.stop(motor_pair.PAIR_1)
        if color_sensor.reflection(sensorE) <= 30:
            motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
            await runloop.until(lambda: color_sensor.color(sensorD)==color_sensor.color(sensorE))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(500)
            return
        virouD = True
    elif dirRefl <= 40:
        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -300, 300)
        await runloop.until(lambda: verSeVirou(-179) or color_sensor.reflection(sensorE)<= 30)
        motor_pair.stop(motor_pair.PAIR_1)
        if color_sensor.reflection(sensorE) <= 30:
            motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
            await runloop.until(lambda: color_sensor.color(sensorD)==color_sensor.color(sensorE))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(500)
            return

        #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
        await runloop.until(lambda: verSeVirou(90) or color_sensor.reflection(sensorD)<= 30)
        motor_pair.stop(motor_pair.PAIR_1)
        if color_sensor.reflection(sensorD) <= 30:
            motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
            await runloop.until(lambda: color_sensor.color(sensorD)==color_sensor.color(sensorE))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(500)
            return
        virouE = True

    #Voltar por onde veio, se não achar linha alguma
    motor_pair.stop(motor_pair.PAIR_1)
    motion_sensor.reset_yaw(0)
    if virouE:
        motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
        await runloop.until(lambda: verSeVirou(90))
    elif virouD:
        motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
        await runloop.until(lambda: verSeVirou(-90))
    seguirLinha()
    await runloop.sleep_ms(100)
    return
def verSeVirou(ang: int):
    if ang > 0:
        return motion_sensor.tilt_angles()[0] >= ang*10
    else:
        return motion_sensor.tilt_angles()[0] <= ang*10

async def darAVolta():
    light_matrix.show_image(light_matrix.IMAGE_GHOST)
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(200)
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
    await runloop.until(lambda: verSeVirou(90))
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(200)
    motor_pair.move_tank(motor_pair.PAIR_1,300,300)
    await runloop.sleep_ms(800)
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(200)
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1,150,400)
    await runloop.until(lambda: color_sensor.color(sensorD)== color.BLACK)
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(200)
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1,300,-120)
    await runloop.until(lambda: verSeVirou(90))
    while not color_sensor.color(sensorD)==color_sensor.color(sensorE):
        seguirLinha()
        await runloop.sleep_ms(10)
    return
#PARTE MAIS IMPORTANTE DO CÓDIGO !!!
runloop.run(main())
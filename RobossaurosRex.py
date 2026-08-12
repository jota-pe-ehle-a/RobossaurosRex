from hub import light_matrix, port, motion_sensor, power_off
import color_sensor
import distance_sensor
import hub
import motor_pair
import color
import runloop

#constantes e coisas a serem declaradas
sensorD = port.F
sensorE = port.E
ultrassonico = port.C
pelaEsquerda = 1
pelaDireita = -1
motor_pair.pair(motor_pair.PAIR_1,port.B,port.A)
kP = 10
areaDeResgate = False
acabou = False
motion_sensor.set_yaw_face(motion_sensor.TOP)

async def main():
    while not acabou:
        if not areaDeResgate and not acabou:
            await foraDaAreaDeResgate()
            continue
        if areaDeResgate and not acabou:
            await dentroDaAreaDeResgate()
            continue
async def foraDaAreaDeResgate():
    global acabou
    seguirLinha()
    #Função pra Parar
    if color_sensor.color(sensorD)== color.RED or color_sensor.color(sensorE)== color.RED:
        motor_pair.stop(motor_pair.PAIR_1)
        acabou = True
        return
    #Função de Contorno
    elif distancia(ultrassonico) <= 40 and distancia(ultrassonico) > 0:
        await darAVolta(pelaEsquerda)
        seguirLinha()
        return
    #Intersecção e/ou Beco Sem Saída
    elif ehVerde(sensorE) or ehVerde(sensorE):
        light_matrix.show_image(light_matrix.IMAGE_TARGET)
        motor_pair.move_tank(motor_pair.PAIR_1,100,100)
        await runloop.sleep_ms(50)
        motor_pair.stop(motor_pair.PAIR_1)
        #Se os dois forem verdes, dá meia volta
        if ehVerde(sensorD) and ehVerde(sensorE):
            await becoSemSaida()
            return
        #Se só o direito for verde, vira à direita
        elif ehVerde(sensorD) and not ehVerde(sensorE):
            await virarADireita()
            return
        #Se só o esquerdo for verde, vira à esquerda
        elif ehVerde(sensorE) and not ehVerde(sensorD):
            await virarAEsquerda()
            return
    #Varredura
    elif (refl(sensorE)< 25 or refl(sensorD)< 25) and abs(motion_sensor.tilt_angles()[1])<50 and abs(motion_sensor.tilt_angles()[2])<50:
        await varredura()
    seguirLinha()
    return

async def dentroDaAreaDeResgate():
    #se não houver nada à frente, continua
    if distancia(ultrassonico) > 60:
        motor_pair.move_tank(motor_pair.PAIR_1,200,200)
        await runloop.sleep_ms(10)
        return
    #se houver algo a frente, para e verifica
    elif distancia(ultrassonico) < 50 and distancia(ultrassonico) > 0:
        motor_pair.move_tank(motor_pair.PAIR_1,-100,-100)
        #recua um pouco
        await runloop.until(lambda: distancia(ultrassonico) >= 60)
        motor_pair.move_tank(motor_pair.PAIR_1,200,-200)
        motion_sensor.reset_yaw(0)
        #vira 90 graus para esquerda
        await runloop.until(lambda: verSeVirou(90))
        #se tiver alguma coisa, vira pro outro lado
        if distancia(ultrassonico) < 80 and distancia(ultrassonico) > 0:
            motor_pair.move_tank(motor_pair.PAIR_1,-200,200)
            motion_sensor.reset_yaw(0)
            #vira 175 graus para direita
            await runloop.until(lambda: verSeVirou(-175))
            #se tiver alguma coisa, volta por onde veio
            if distancia(ultrassonico) < 80 and distancia(ultrassonico) > 0:
                motor_pair.move_tank(motor_pair.PAIR_1,-200,200)
                motion_sensor.reset_yaw(0)
                await runloop.until(lambda: verSeVirou(-90))
                motor_pair.move_tank(motor_pair.PAIR_1,200,200)
                await runloop.sleep_ms(10)
                return
            #senão, continua
            else:
                motor_pair.move_tank(motor_pair.PAIR_1,200,200)
                await runloop.sleep_ms(10)
                return
        #senão, continua
        else:
            motor_pair.move_tank(motor_pair.PAIR_1,200,200)
            await runloop.sleep_ms(10)
            return

def seguirLinha():
    #Atribuição de potência com base na diferença de reflexão entre os sensores
    light_matrix.show_image(light_matrix.IMAGE_ARROW_N)
    erro = (color_sensor.reflection(sensorE)-color_sensor.reflection(sensorD))*kP
    powerD = min(max(-1000,300 + erro),1000)
    powerE = min(max(-1000,300 - erro),1000)
    motor_pair.move_tank(motor_pair.PAIR_1,powerE,powerD)


async def varredura():
    light_matrix.show_image(light_matrix.IMAGE_CHESSBOARD)
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,300,300,100)
    esqRefl = ehPreto(sensorE)
    dirRefl = ehPreto(sensorD)
    #Ver se é uma intersecção, se sim passar direto
    motor_pair.move_tank(motor_pair.PAIR_1,300,300)
    await runloop.sleep_ms(400)
    motor_pair.stop(motor_pair.PAIR_1)
    if ehVerde(sensorE) or ehVerde(sensorD):
        motor_pair.move_tank(motor_pair.PAIR_1,300,300)
        await runloop.sleep_ms(300)
        seguirLinha()
        return

    if esqRefl:
        #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
        await runloop.until(lambda: verSeVirou(90) or ehPreto(sensorD))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorD):
            motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            seguirLinha()
            return
        await runloop.sleep_ms(10)
        
        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -300, 300)
        await runloop.until(lambda: verSeVirou(-175) or ehPreto(sensorE))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorE):
            motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            seguirLinha()
            return

    elif dirRefl:
        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -300, 300)
        await runloop.until(lambda: verSeVirou(-90) or ehPreto(sensorE))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorE):
            motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            seguirLinha()
            return

        #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
        await runloop.until(lambda: verSeVirou(175) or ehPreto(sensorD))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorD):
            motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            seguirLinha()
            return

#função booleana que retorna se o sensor virou o ângulo
def verSeVirou(ang: int):
    if ang > 0:
        return motion_sensor.tilt_angles()[0] >= ang*10
    else:
        return motion_sensor.tilt_angles()[0] <= ang*10

#função booleana que retorna se um sensor está vendo preto
def ehPreto(sensor: int):
    return color_sensor.color(sensor) == color.BLACK

#função booleana que retorna se um sensor está vendo verde
def ehVerde(sensor: int):
    return color_sensor.color(sensor) == color.GREEN

#função que retorna o valor da reflexão 
def refl(sensor: int):
    return color_sensor.reflection(sensor)

#função que retorna o valor da distância, em mm
def distancia(sensor: int):
    return distance_sensor.distance(sensor)

# resolver o obstáculo
async def darAVolta(direcao: int):
    light_matrix.show_image(light_matrix.IMAGE_GHOST)
    motor_pair.move_tank(motor_pair.PAIR_1,-100,-100)
    await runloop.until(lambda: distance_sensor.distance(port.C)>=60)
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1,300*direcao,-300*direcao)
    await runloop.until(lambda: verSeVirou(88*direcao))
    motor_pair.move_tank(motor_pair.PAIR_1,300,300)
    await runloop.sleep_ms(500)
    motion_sensor.reset_yaw(0)
    if direcao == pelaEsquerda:
        motor_pair.move_tank(motor_pair.PAIR_1,170,450)
        await runloop.until(lambda: color_sensor.color(sensorE)== color.BLACK)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,300,300)
        await runloop.sleep_ms(600)
        motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
        await runloop.until(lambda: verSeVirou(90) or color_sensor.color(sensorD)== color.BLACK)
        motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
    elif direcao == pelaDireita:
        motor_pair.move_tank(motor_pair.PAIR_1,450,170)
        await runloop.until(lambda: color_sensor.color(sensorD)== color.BLACK)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,300,300)
        await runloop.sleep_ms(600)
        motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
        await runloop.until(lambda: verSeVirou(-90) or color_sensor.color(sensorD)== color.BLACK)
        motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
    await runloop.until(lambda: color_sensor.color(sensorE)==color_sensor.color(sensorD))
    return

async def becoSemSaida():
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
    await runloop.until(lambda: verSeVirou(120))
    await runloop.until(lambda: color_sensor.color(sensorD) == color.BLACK)
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,100,100,200)
    motor_pair.move_tank(motor_pair.PAIR_1,-200,200)
    await runloop.until(lambda: color_sensor.reflection(sensorD)==color_sensor.reflection(sensorE))
    seguirLinha()
    return

#Função de Intersecção à Direita
async def virarADireita():
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,300,300,300)
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1,-150,300)
    await runloop.until(lambda: verSeVirou(-45))
    await runloop.until(lambda: verSeVirou(-90) or color_sensor.color(sensorE)== color.BLACK)
    motor_pair.move_tank(motor_pair.PAIR_1,300,-300)
    await runloop.until(lambda: color_sensor.reflection(sensorD)==color_sensor.reflection(sensorE))
    seguirLinha()
    return

#Função de Intersecção à Esquerda
async def virarAEsquerda():
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,300,300,300)
    motion_sensor.reset_yaw(0)
    motor_pair.move_tank(motor_pair.PAIR_1,300,-150)
    await runloop.until(lambda: verSeVirou(45))
    await runloop.until(lambda: verSeVirou(90) or color_sensor.color(sensorD)== color.BLACK)
    motor_pair.move_tank(motor_pair.PAIR_1,-300,300)
    await runloop.until(lambda: color_sensor.reflection(sensorD)==color_sensor.reflection(sensorE))
    seguirLinha()
    return
#PARTE MAIS IMPORTANTE DO CÓDIGO !!!
runloop.run(main())

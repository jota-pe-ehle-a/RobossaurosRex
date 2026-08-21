from hub import light_matrix, port, motion_sensor
import color_sensor
import distance_sensor
import motor_pair
import color
import runloop

#constantes e coisas a serem declaradas
sensorD = port.F
sensorE = port.E
ultrassonico = port.C
pelaEsquerda = 1
pelaDireita = -1
velBase = 300
velDevagar = int(velBase/3)
motor_pair.pair(motor_pair.PAIR_1,port.B,port.A)
kP = 16
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
    global kP

    #se ele estiver num plano inclinado, a constante proporcional vai diminuir par evitar erros
    if (abs(motion_sensor.tilt_angles()[1])>100 or abs(motion_sensor.tilt_angles()[2])>100) and kP == 16:
        kP = 4
    #quando ele volar para o plano horizontal, a constante proporcional aumenta novamente
    elif  (abs(motion_sensor.tilt_angles()[1])<50 or abs(motion_sensor.tilt_angles()[2])<50) and kP == 4:
        kP = 16

    seguirLinha()
    #Função pra Parar
    if color_sensor.color(sensorD) == color.RED or color_sensor.color(sensorE) == color.RED:
        motor_pair.stop(motor_pair.PAIR_1)
        acabou = True
        return
    #Função de Contorno
    elif distancia(ultrassonico) <= 50 and distancia(ultrassonico) > 0 and abs(motion_sensor.tilt_angles()[1])<50:
        await darAVolta(pelaEsquerda)
        seguirLinha()
        return
    #Intersecção e/ou Beco Sem Saída
    elif ehVerde(sensorD) or ehVerde(sensorE):
        light_matrix.show_image(light_matrix.IMAGE_TARGET)
        motor_pair.move_tank(motor_pair.PAIR_1,velDevagar,velDevagar)
        await runloop.sleep_ms(200)
        motor_pair.stop(motor_pair.PAIR_1)
        #Se os dois forem verdes, dá meia volta
        if ehVerde(sensorD) and ehVerde(sensorE):
            await becoSemSaida()
            return
        #Se só o direito for verde, vira à direita
        elif ehVerde(sensorD):
            await virarADireita()
            return
        #Se só o esquerdo for verde, vira à esquerda
        elif ehVerde(sensorE):
            await virarAEsquerda()
            return
    #Varredura
    elif (refl(sensorE)< 24 or refl(sensorD)< 24) and abs(motion_sensor.tilt_angles()[1])<30 and abs(motion_sensor.tilt_angles()[2])<30:
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
    powerD = min(max(-1000,velBase + erro),1000)
    powerE = min(max(-1000,velBase - erro),1000)
    motor_pair.move_tank(motor_pair.PAIR_1,powerE,powerD)


async def varredura():
    light_matrix.show_image(light_matrix.IMAGE_CHESSBOARD)
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,velBase,velBase,100)
    #Conserva os valores de reflexão de ambos os sensores
    esqRefl = refl(sensorE)
    dirRefl = refl(sensorD)
    #Ver se é uma intersecção, se sim passar direto
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
    await runloop.sleep_ms(300)
    if ehVerde(sensorE) or ehVerde(sensorD):
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(300)
        seguirLinha()
        return
    await runloop.sleep_ms(200)

    if esqRefl<=dirRefl:
        #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: ehPreto(sensorD) or verSeVirou(88))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorD):
            motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            seguirLinha()
            return
        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -velBase, velBase)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: verSeVirou(-175) or ehPreto(sensorE))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorE):
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            seguirLinha()
            return

    elif dirRefl<esqRefl:
        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -velBase, velBase)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: ehPreto(sensorE) or verSeVirou(-88))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorE):
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            seguirLinha()
            return

        #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: verSeVirou(175) or ehPreto(sensorD))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorD):
            motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
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
    #volta um pouco pra trás para ajustar a posição
    light_matrix.show_image(light_matrix.IMAGE_GHOST)
    motor_pair.move_tank(motor_pair.PAIR_1,-velDevagar,-velDevagar)
    await runloop.until(lambda: distance_sensor.distance(port.C)>=65)
    motor_pair.stop(motor_pair.PAIR_1)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    #vira para um dos lados dependendo da direção fornecida
    motor_pair.move_tank(motor_pair.PAIR_1,velBase*direcao,-velBase*direcao)
    await runloop.until(lambda: verSeVirou(85*direcao))
    #anda um pouco pra frente para ajustar a posição
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
    await runloop.sleep_ms(300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    
    if direcao == pelaEsquerda:
        #a diferença de potência entre os motores serve para que o robô ande curvado para direita
        motor_pair.move_tank(motor_pair.PAIR_1,170,450)
        #até ver preto
        await runloop.until(lambda: ehPreto(sensorE) or ehPreto(sensorD))
        motion_sensor.reset_yaw(0)
        #vai um pouco pra frente para ajustar a posição
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(500)
        #gira  para a esquerda até que o robô alcance a linha
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        await runloop.until(lambda: verSeVirou(70) or ehPreto(sensorD))
        motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
        await runloop.sleep_ms(100)

    elif direcao == pelaDireita:
        #a diferença de potência entre os motores serve para que o robô ande curvado para esquerda
        motor_pair.move_tank(motor_pair.PAIR_1,450,170)
        #até ver preto
        await runloop.until(lambda: ehPreto(sensorD) or ehPreto(sensorE))
        motion_sensor.reset_yaw(0)
        #vai um pouco pra frente pra ajustar a posição
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(500)
        #gira para a direita até que o robô alcance a linha
        motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
        await runloop.until(lambda: verSeVirou(-70) or ehPreto(sensorE))
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        await runloop.sleep_ms(100)

    #alinhamento final
    await runloop.until(lambda: color_sensor.color(sensorE)==color_sensor.color(sensorD))
    return

async def becoSemSaida():
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
    await runloop.until(lambda: verSeVirou(120))
    await runloop.until(lambda: ehPreto(sensorD))
    motor_pair.move_tank(motor_pair.PAIR_1,-2*velDevagar,2*velDevagar)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    seguirLinha()
    return

#Função de Intersecção à Direita
async def virarADireita():
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,300,300,300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,int(-velBase/2),velBase)
    await runloop.until(lambda: verSeVirou(-45))
    await runloop.until(lambda: verSeVirou(-90) or ehPreto(sensorE))
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velDevagar)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    seguirLinha()
    return

#Função de Intersecção à Esquerda
async def virarAEsquerda():
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,300,300,300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,int(-velBase/2))
    await runloop.until(lambda: verSeVirou(45))
    await runloop.until(lambda: verSeVirou(90) or ehPreto(sensorD))
    motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    seguirLinha()
    return

#PARTE MAIS IMPORTANTE DO CÓDIGO !!!
runloop.run(main())

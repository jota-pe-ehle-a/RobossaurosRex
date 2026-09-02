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
lateral = port.D
pelaEsquerda = 1
pelaDireita = -1
velBase = 300
velDevagar = int(velBase/2)
motor_pair.pair(motor_pair.PAIR_1,port.B,port.A)
kP = 16
areaDeResgate = False
acabou = False
entrou = False
motion_sensor.set_yaw_face(motion_sensor.TOP)

async def main():
    while not acabou:
        global areaDeResgate
        if not areaDeResgate and not acabou:
            await foraDaAreaDeResgate()
            continue
        if areaDeResgate and not acabou:
            await dentroDaAreaDeResgate()
            continue
async def foraDaAreaDeResgate():
    global areaDeResgate
    global acabou
    global kP

    #se ele estiver num plano inclinado, a constante proporcional vai diminuir par evitar erros
    if (abs(motion_sensor.tilt_angles()[1])>50 or abs(motion_sensor.tilt_angles()[2])>50) and kP == 16:
        kP = 3
        return
    #quando ele volar para o plano horizontal, a constante proporcional aumenta novamente
    elif  (abs(motion_sensor.tilt_angles()[1])<50 and abs(motion_sensor.tilt_angles()[2])<50) and kP == 3:
        kP = 16
        return
    if ehPrata(sensorD) and ehPrata(sensorE) and abs(motion_sensor.tilt_angles()[1]) < 20 and abs(motion_sensor.tilt_angles()[2]) < 20:
        seguirLinha()
        await runloop.until(lambda: color_sensor.rgbi(sensorD)[0]>700)
        motor_pair.stop(motor_pair.PAIR_1)
        await runloop.sleep_ms(300)
        areaDeResgate = True
        return

    seguirLinha()
    #Função pra Parar
    if color_sensor.color(sensorD) == color.RED or color_sensor.color(sensorE) == color.RED:
        motor_pair.stop(motor_pair.PAIR_1)
        acabou = True
        return
    seguirLinha()
    #Função de Contorno
    if (distancia(ultrassonico) <= 50 and distancia(ultrassonico) > 0) and abs(motion_sensor.tilt_angles()[1])<30:
        await darAVolta(pelaEsquerda)
        seguirLinha()
        return
    seguirLinha()
    #Intersecção e/ou Beco Sem Saída
    if ehVerde(sensorD) or ehVerde(sensorE):
        light_matrix.show_image(light_matrix.IMAGE_TARGET)
        motor_pair.move_tank(motor_pair.PAIR_1,velDevagar,velDevagar)
        await runloop.sleep_ms(100)
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
    seguirLinha()
    #Varredura
    if (refl(sensorE)< 25 or refl(sensorD)< 25) and abs(motion_sensor.tilt_angles()[1])<15 and abs(motion_sensor.tilt_angles()[2])<15:
        await varredura()
    seguirLinha()
    return

async def dentroDaAreaDeResgate():
    light_matrix.show_image(light_matrix.IMAGE_GO_UP)
    global areaDeResgate
    #verifica se o robô já alcançou a saída
    if ehPreto(sensorD) or ehPreto(sensorE):
        seguirLinha()
        await runloop.sleep_ms(100)
        areaDeResgate = False
        return
    #aqui ele registra as distancias dos sensores
    distanciaFrente = distancia(ultrassonico) if distancia(ultrassonico) > 0 else 1000
    distanciaLateral = distancia(lateral) if distancia(lateral) > 0 else 1000

    #se houver parede à frente e parede à esquerda:
    if distanciaFrente < 300 and distanciaLateral < 150:
        #contornar o ladrilho a frente
        light_matrix.show_image(light_matrix.IMAGE_GO_RIGHT)
        await girarAngulo(-85)
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,1200,velBase,velBase)
        light_matrix.show_image(light_matrix.IMAGE_GO_LEFT)
        await girarAngulo(85)
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,1000,velBase,velBase)
        distanciaFrente = distancia(ultrassonico) if distancia(ultrassonico) > 0 else 1000
        distanciaLateral = distancia(lateral) if distancia(lateral) > 0 else 1000
        #se não houver nada à frente, achou a saída e deve seguir
        if distanciaFrente > 100:
            light_matrix.show_image(light_matrix.IMAGE_GO_UP)
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
            await runloop.until(lambda: ehPreto(sensorD) and ehPreto(sensorE))
            return
        #se houver parede à frente e não houver parede à esquerda do ladrilho contornado, vira e segue
        elif distanciaFrente < 100 and distanciaLateral > 380:
            light_matrix.show_image(light_matrix.IMAGE_GO_LEFT)
            await girarAngulo(85)
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
            await runloop.until(lambda: ehPreto(sensorD) and ehPreto(sensorE))
            return
        #se houver parede tanto à frente quanto à esquerda do ladrilho contornado, vira à direita e continua seguindo a parede
        elif distanciaFrente < 100 and distanciaLateral < 380:
            light_matrix.show_image(light_matrix.IMAGE_GO_RIGHT)
            await girarAngulo(-85)
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
            return
    #se não houver parede à frente e houver parede à esquerda:
    elif distanciaFrente > 280 and distanciaLateral < 150:
        #vai para frente 
        light_matrix.show_image(light_matrix.IMAGE_GO_UP)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        return
    #se houver parede à frente e não houver parede à esquerda:
    elif distanciaFrente < 150 and distanciaLateral > 150:
        #gira para a direita e segue
        light_matrix.show_image(light_matrix.IMAGE_GO_RIGHT)
        await girarAngulo(-85)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        return
    #se não houver nada à frente nem à esquerda:
    else: 
        #vai um pouco para frente para se alinhar
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,300, velBase,velBase)
        #virar à esquerda e segue
        light_matrix.show_image(light_matrix.IMAGE_GO_LEFT)
        await girarAngulo(88)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.until(lambda: ehPreto(sensorD) or ehPreto(sensorE) or distancia(ultrassonico) < 120)
        return


def seguirLinha():
    #Atribuição de potência com base na diferença de reflexão entre os sensores
    light_matrix.show_image(light_matrix.IMAGE_ARROW_N)
    erro = (refl(sensorE)-refl(sensorD))*kP
    powerD = min(max(-1000,velBase + erro),1000)
    powerE = min(max(-1000,velBase - erro),1000)
    motor_pair.move_tank(motor_pair.PAIR_1,powerE,powerD)

async def girarAngulo(ang: int):
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    while not verSeVirou(ang):
        motor_pair.move_tank(motor_pair.PAIR_1,
        int(velBase*((abs(motion_sensor.tilt_angles()[0]/10-ang)+30)/ang)),
        int(-velBase*((abs(motion_sensor.tilt_angles()[0]/10-ang)+30)/ang)))

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

#função booleana que retorna se um sensor está vendo prata
def ehPrata(sensor: int):
    return color_sensor.color(sensor) == color.WHITE and refl(sensor) >= 99 and estaEntre(color_sensor.rgbi(sensor)[0],530,610) and estaEntre(color_sensor.rgbi(sensor)[1],570,650) and estaEntre(color_sensor.rgbi(sensor)[2],590,670)

#função booleana que retorna se um número está no intervalo selecionado:
def estaEntre(n: int, minimo: int, maximo: int):
    return n > minimo and n < maximo

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
    await runloop.until(lambda: distance_sensor.distance(port.C)>=60)
    motor_pair.stop(motor_pair.PAIR_1)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    #vira para um dos lados dependendo da direção fornecida
    motor_pair.move_tank(motor_pair.PAIR_1,velBase*direcao,-velBase*direcao)
    await runloop.until(lambda: verSeVirou(88*direcao))
    #anda um pouco pra frente para ajustar a posição
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
    await runloop.sleep_ms(300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    
    if direcao == pelaEsquerda:
        #a diferença de potência entre os motores serve para que o robô ande curvado para direita
        motor_pair.move_tank(motor_pair.PAIR_1,175,450)
        await runloop.sleep_ms(100)
        #até ver preto
        await runloop.until(lambda: ehPreto(sensorE) or ehPreto(sensorD) or verSeVirou(-170))
        motion_sensor.reset_yaw(0)
        #vai um pouco pra frente para ajustar a posição
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(500)
        #gira  para a esquerda até que o robô alcance a linha
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        await runloop.until(lambda: verSeVirou(90) or ehPreto(sensorD))
        motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
        await runloop.sleep_ms(100)

    elif direcao == pelaDireita:
        #a diferença de potência entre os motores serve para que o robô ande curvado para esquerda
        motor_pair.move_tank(motor_pair.PAIR_1,450,175)
        await runloop.sleep_ms(100)
        #até ver preto
        await runloop.until(lambda: ehPreto(sensorD) or ehPreto(sensorE) or verSeVirou(170))
        motion_sensor.reset_yaw(0)
        #vai um pouco pra frente pra ajustar a posição
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(500)
        #gira para a direita até que o robô alcance a linha
        motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
        await runloop.until(lambda: verSeVirou(-90) or ehPreto(sensorE))
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
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,velBase,velBase,300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,-velDevagar,velBase)
    await runloop.until(lambda: verSeVirou(-45))
    await runloop.until(lambda: verSeVirou(-90) or ehPreto(sensorE))
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velDevagar)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    seguirLinha()
    return

#Função de Intersecção à Esquerda
async def virarAEsquerda():
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,velBase,velBase,300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velDevagar)
    await runloop.until(lambda: verSeVirou(45))
    await runloop.until(lambda: verSeVirou(90) or ehPreto(sensorD))
    motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    seguirLinha()
    return

#PARTE MAIS IMPORTANTE DO CÓDIGO !!!
runloop.run(main())

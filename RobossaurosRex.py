from hub import light_matrix, port, motion_sensor, sound
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
iniciou = False
velBase = 300
velDevagar = int(velBase/2)
motor_pair.pair(motor_pair.PAIR_1,port.B,port.A)
kP = 16
areaDeResgate = False
acabou = False
motion_sensor.set_yaw_face(motion_sensor.TOP)

async def main():
    i = 50
    contador = 0
    global areaDeResgate
    global acabou

    while not acabou:
        if contador > 400:
            i = -i
            contador = 0
        contador += 1
        distance_sensor.show(ultrassonico, [50+i,50+i,50-i,50-i])
        if not areaDeResgate and not acabou:
            await foraDaAreaDeResgate()
            continue
        if areaDeResgate and not acabou:
            await dentroDaAreaDeResgate()
            continue
    return
async def foraDaAreaDeResgate():
    global areaDeResgate
    global acabou
    global kP
    

    #se ele estiver num plano inclinado, a constante proporcional vai diminuir par evitar erros
    if (abs(motion_sensor.tilt_angles()[1])>50 or abs(motion_sensor.tilt_angles()[2])>50) and kP == 16:
        kP = 1
        return
    #quando ele volar para o plano horizontal, a constante proporcional aumenta novamente
    elif  (abs(motion_sensor.tilt_angles()[1])<50 and abs(motion_sensor.tilt_angles()[2])<50) and kP == 1:
        kP = 16
        return
    if ehPrata(sensorD) and ehPrata(sensorE):
        light_matrix.show_image(light_matrix.IMAGE_GO_UP)
        await seguirLinha()
        runloop.sleep_ms(10)
        motor_pair.stop(motor_pair.PAIR_1)
        await runloop.until(lambda: ehPrata(sensorD) and ehPrata(sensorE))
        await seguirLinha()
        await runloop.until(lambda: color_sensor.rgbi(sensorD)[0]>700)
        motor_pair.stop(motor_pair.PAIR_1)
        await runloop.sleep_ms(300)
        areaDeResgate = True
        await alinharSe()
        return
    if color_sensor.color(sensorE) == color.WHITE and color_sensor.color(sensorD) == color.WHITE:
        await seguirLinha()
        return

    #Função pra Parar
    if color_sensor.color(sensorD) == color.RED or color_sensor.color(sensorE) == color.RED:
        distance_sensor.clear(ultrassonico)
        motor_pair.stop(motor_pair.PAIR_1)
        sound.beep(800,2000)
        acabou = True
        return
    #Função de Contorno
    if distancia(ultrassonico) <= 40 and distancia(ultrassonico) > 0 and abs(motion_sensor.tilt_angles()[1])<30:
        await darAVolta(pelaDireita)
        await seguirLinha()
        return
    #Intersecção e/ou Beco Sem Saída
    if ehVerde(sensorD) or ehVerde(sensorE):
        distance_sensor.clear(ultrassonico)
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
        
    #Busca-Busca
    if (refl(sensorE)< 25 or refl(sensorD)< 25) and abs(motion_sensor.tilt_angles()[1])<20 and abs(motion_sensor.tilt_angles()[2])<20:
        await buscaBusca()
        await seguirLinha()
        return
    return

async def dentroDaAreaDeResgate():
    light_matrix.show_image(light_matrix.IMAGE_GO_UP)
    global areaDeResgate
    #verifica se o robô já alcançou a saída
    if ehPreto(sensorD) or ehPreto(sensorE):
        await seguirLinha()
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
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,1100,velBase,velBase)
        light_matrix.show_image(light_matrix.IMAGE_GO_LEFT)
        await girarAngulo(88)
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,950,velBase,velBase)
        distanciaFrente = distancia(ultrassonico) if distancia(ultrassonico) > 0 else 1000
        distanciaLateral = distancia(lateral) if distancia(lateral) > 0 else 1000
        #se não houver nada à frente, achou a saída e deve seguir
        if distanciaFrente > 150:
            light_matrix.show_image(light_matrix.IMAGE_GO_UP)
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
            await runloop.until(lambda: ehPreto(sensorD) and ehPreto(sensorE))
            return
        #se houver parede à frente e não houver parede à esquerda do ladrilho contornado, vira e segue
        elif distanciaFrente < 150 and distanciaLateral > 420:
            light_matrix.show_image(light_matrix.IMAGE_GO_LEFT)
            await girarAngulo(85)
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
            await runloop.until(lambda: ehPreto(sensorD) and ehPreto(sensorE))
            return
        #se houver parede tanto à frente quanto à esquerda do ladrilho contornado, vira à direita e continua seguindo a parede
        elif distanciaFrente < 150 and distanciaLateral < 420:
            light_matrix.show_image(light_matrix.IMAGE_GO_RIGHT)
            await girarAngulo(-88)
            motion_sensor.reset_yaw(0)
            andarFrente()
            return
    #se não houver parede à frente e houver parede à esquerda:
    elif distanciaFrente > 280 and distanciaLateral < 150:
        #vai para frente 
        light_matrix.show_image(light_matrix.IMAGE_GO_UP)
        andarFrente()
        return
    #se houver parede à frente e não houver parede à esquerda:
    elif distanciaFrente < 150 and distanciaLateral > 150:
        #gira para a direita e segue
        light_matrix.show_image(light_matrix.IMAGE_GO_RIGHT)
        await girarAngulo(-85)
        motion_sensor.reset_yaw(0)
        andarFrente()
        return
    #se não houver nada à frente nem à esquerda:
    else: 
        #vai um pouco para frente para se alinhar
        await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,300, velBase,velBase)
        #virar à esquerda e segue
        light_matrix.show_image(light_matrix.IMAGE_GO_LEFT)
        await girarAngulo(85)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.until(lambda: ehPreto(sensorD) or ehPreto(sensorE) or distancia(ultrassonico) < 500)
        motion_sensor.reset_yaw(0)
        return

async def alinharSe():
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,500,velBase,velBase)
    distanciaFrente = distancia(ultrassonico) if distancia(ultrassonico) > 0 else 1000
    distanciaLateral = distancia(lateral) if distancia(lateral) > 0 else 1000
    menorFrente = distanciaFrente
    menorLado = distanciaLateral
    motor_pair.move_tank(motor_pair.PAIR_1, -velDevagar, velDevagar)
    while True:
        distanciaFrente = distancia(ultrassonico) if distancia(ultrassonico) > 0 else 1000
        distanciaLateral = distancia(lateral) if distancia(lateral) > 0 else 1000
        if distanciaFrente < menorFrente:
            menorFrente = distanciaFrente
        if distanciaLateral < menorLado:
            menorLado = distanciaLateral
        if distanciaLateral > menorLado or distanciaFrente > menorFrente:
            motor_pair.move_tank(motor_pair.PAIR_1, velDevagar, -velDevagar)
            await runloop.until(lambda: distancia(ultrassonico)  == menorFrente or distancia(lateral) == menorLado)
            motor_pair.stop(motor_pair.PAIR_1)
            motion_sensor.reset_yaw(0)
            await runloop.sleep_ms(10)
            return
    

async def seguirLinha():
    #Atribuição de potência com base na diferença de reflexão entre os sensores
    light_matrix.show_image(light_matrix.IMAGE_ARROW_N)
    erro = (refl(sensorE)-refl(sensorD))*kP
    powerD = min(max(-1000,velBase + erro),1000)
    powerE = min(max(-1000,velBase - erro),1000)
    motor_pair.move_tank(motor_pair.PAIR_1,powerE,powerD)
    await runloop.sleep_ms(10)

async def girarAngulo(ang: int):
    motion_sensor.reset_yaw(0)  
    await runloop.sleep_ms(10)
    while not verSeVirou(ang):
        motor_pair.move_tank(motor_pair.PAIR_1,
        int(velBase*((abs(motion_sensor.tilt_angles()[0]/10-ang)+30)/ang)),
        int(-velBase*((abs(motion_sensor.tilt_angles()[0]/10-ang)+30)/ang)))

async def buscaBusca():
    distance_sensor.clear(ultrassonico)
    light_matrix.show_image(light_matrix.IMAGE_CHESSBOARD)
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,20,velBase,velBase)
    #Conserva os valores de reflexão de ambos os sensores
    esqRefl = refl(sensorE)
    dirRefl = refl(sensorD)
    #Ver se é uma intersecção, se sim passar direto
    await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,90,velBase,velBase)
    if ehVerde(sensorE) or ehVerde(sensorD):
        motor_pair.stop(motor_pair.PAIR_1)
        light_matrix.show_image(light_matrix.IMAGE_TARGET)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(300)
        await seguirLinha()
        return

    if esqRefl<=dirRefl:
        #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        darSeta(pelaEsquerda)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: ehPreto(sensorD) or verSeVirou(88))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorD):
            motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            await seguirLinha()
            return
        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -velBase, velBase)
        darSeta(pelaDireita)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: verSeVirou(-175) or ehPreto(sensorE))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorE):
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            await seguirLinha()
            return
        #se não houver linha, vai pra frente
        await girarAngulo(85)

    elif dirRefl<esqRefl:
        #Girar para a direita para procurar linha, se não houver na esquerda
        light_matrix.show_image(light_matrix.IMAGE_ARROW_E)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1, -velBase, velBase)
        darSeta(pelaDireita)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: ehPreto(sensorE) or verSeVirou(-88))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorE):
            motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            await seguirLinha()
            return

        #Girar para a esquerda para procurar linha, se não for intersecção
        light_matrix.show_image(light_matrix.IMAGE_ARROW_W)
        motion_sensor.reset_yaw(0)
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        darSeta(pelaDireita)
        await runloop.sleep_ms(10)
        await runloop.until(lambda: verSeVirou(175) or ehPreto(sensorD))
        motor_pair.stop(motor_pair.PAIR_1)
        if ehPreto(sensorD):
            motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
            await runloop.until(lambda: refl(sensorE)==refl(sensorD))
            motor_pair.stop(motor_pair.PAIR_1)
            await runloop.sleep_ms(200)
            await seguirLinha()
            return
        #se não houver linha, vai pra frente
        await girarAngulo(-85)


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
    return color_sensor.color(sensor) == color.WHITE and estaEntre(refl(sensor),93,100) and estaEntre(color_sensor.rgbi(sensor)[0],1000,1030) and estaEntre(color_sensor.rgbi(sensor)[1],1010,1030) and estaEntre(color_sensor.rgbi(sensor)[2],1010,1030) 

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
    distance_sensor.clear(ultrassonico)
    #volta um pouco pra trás para ajustar a posição
    light_matrix.show_image(light_matrix.IMAGE_GHOST)
    motor_pair.move_tank(motor_pair.PAIR_1,velDevagar,velDevagar)
    await runloop.sleep_ms(100)
    if not (distancia(ultrassonico) <= 40 and distancia(ultrassonico) > 0):
        return
    motor_pair.move_tank(motor_pair.PAIR_1,-velDevagar,-velDevagar)
    await runloop.until(lambda: distance_sensor.distance(port.C)>=60)
    motor_pair.stop(motor_pair.PAIR_1)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    #vira para um dos lados dependendo da direção fornecida
    darSeta(-direcao)
    await girarAngulo(85*direcao)
    #anda um pouco pra frente para ajustar a posição
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
    await runloop.sleep_ms(300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    
    if direcao == pelaEsquerda:
        #a diferença de potência entre os motores serve para que o robô ande curvado para direita
        motor_pair.move_tank(motor_pair.PAIR_1,150,450)
        await runloop.sleep_ms(100)
        #até ver preto
        await runloop.until(lambda: ehPreto(sensorE) or ehPreto(sensorD) or verSeVirou(178))
        motion_sensor.reset_yaw(0)
        #vai um pouco pra frente para ajustar a posição
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(500)
        #gira  para a esquerda até que o robô alcance a linha
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        await runloop.until(lambda: verSeVirou(81) or ehPreto(sensorD))
        motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
        await runloop.sleep_ms(100)

    elif direcao == pelaDireita:
        #a diferença de potência entre os motores serve para que o robô ande curvado para esquerda
        motor_pair.move_tank(motor_pair.PAIR_1,450,150)
        await runloop.sleep_ms(100)
        #até ver preto
        await runloop.until(lambda: ehPreto(sensorD) or ehPreto(sensorE) or verSeVirou(-178))
        motion_sensor.reset_yaw(0)
        #vai um pouco pra frente pra ajustar a posição
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,velBase)
        await runloop.sleep_ms(500)
        #gira para a direita até que o robô alcance a linha
        motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
        await runloop.until(lambda: verSeVirou(-81) or ehPreto(sensorE))
        motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
        await runloop.sleep_ms(100)

    #alinhamento final
    await runloop.until(lambda: color_sensor.color(sensorE)==color_sensor.color(sensorD))
    return

async def becoSemSaida():
    light_matrix.show_image(light_matrix.IMAGE_SQUARE)
    distance_sensor.show(ultrassonico,[100]*4)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velBase)
    await runloop.until(lambda: verSeVirou(120))
    await runloop.until(lambda: ehPreto(sensorD))
    motor_pair.move_tank(motor_pair.PAIR_1,-2*velDevagar,2*velDevagar)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    await seguirLinha()
    return

#Função de Intersecção à Direita
async def virarADireita():
    motor_pair.stop(motor_pair.PAIR_1)
    light_matrix.show_image(light_matrix.IMAGE_TARGET)
    darSeta(pelaDireita)
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,velBase,velBase,300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,-velDevagar,velBase)
    await runloop.until(lambda: verSeVirou(-45))
    await runloop.until(lambda: verSeVirou(-90) or ehPreto(sensorE))
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velDevagar)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    await seguirLinha()
    return

#Função de Intersecção à Esquerda
async def virarAEsquerda():
    motor_pair.stop(motor_pair.PAIR_1)
    light_matrix.show_image(light_matrix.IMAGE_TARGET)
    darSeta(pelaEsquerda)
    await motor_pair.move_tank_for_time(motor_pair.PAIR_1,velBase,velBase,300)
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(10)
    motor_pair.move_tank(motor_pair.PAIR_1,velBase,-velDevagar)
    await runloop.until(lambda: verSeVirou(45))
    await runloop.until(lambda: verSeVirou(90) or ehPreto(sensorD))
    motor_pair.move_tank(motor_pair.PAIR_1,-velBase,velBase)
    await runloop.until(lambda: refl(sensorD) == refl(sensorE))
    await seguirLinha()
    return

def darSeta(direcao: int):
    distance_sensor.show(ultrassonico,[50-50*direcao,50+50*direcao,50-50*direcao,50+50*direcao])

def andarFrente():
    global kP
    ang = motion_sensor.tilt_angles()[0]/10
    powerEsq = int(velBase - kP*ang)
    powerDir = int(velBase + kP*ang)
    motor_pair.move_tank(motor_pair.PAIR_1,powerEsq,powerDir)

#PARTE MAIS IMPORTANTE DO CÓDIGO !!!
runloop.run(main())

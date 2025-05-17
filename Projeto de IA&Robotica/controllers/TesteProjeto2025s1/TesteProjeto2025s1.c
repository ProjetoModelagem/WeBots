
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#define TIME_STEP 32
#define QtddCaixas 20
#define QtddSensores 8

#define LIMIAR_MOVIMENTO_CAIXA 0.015
#define LIMIAR_PAREDE 120.0
#define VELOCIDADE 6.0
#define TEMPO_INICIAL_IGNORADO 50
#define LIMIAR_TRAVADO 0.005
#define CICLOS_TRAVADO 62

int main() {
  wb_robot_init();
  srand(time(NULL));

  WbDeviceTag MotorEsquerdo = wb_robot_get_device("left wheel motor");
  WbDeviceTag MotorDireito = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  WbDeviceTag sensores[QtddSensores];
  char nomeSensor[5];
  for (int i = 0; i < QtddSensores; i++) {
    sprintf(nomeSensor, "ps%d", i);
    sensores[i] = wb_robot_get_device(nomeSensor);
    wb_distance_sensor_enable(sensores[i], TIME_STEP);
  }

  WbNodeRef caixaLeve = NULL;
  double posicaoInicialX = 0.0, posicaoInicialZ = 0.0;
  int posicaoSalva = 0;

  for (int i = 1; i <= QtddCaixas; i++) {
    char nomeCaixa[10];
    sprintf(nomeCaixa, "CAIXA%02d", i);
    WbNodeRef caixa = wb_supervisor_node_get_from_def(nomeCaixa);
    if (caixa != NULL) {
      WbFieldRef campoMassa = wb_supervisor_node_get_field(caixa, "mass");
      if (campoMassa) {
        double massa = wb_supervisor_field_get_sf_float(campoMassa);
        if (massa == 0.06) {
          caixaLeve = caixa;
          printf("Caixa leve encontrada: %s\n", nomeCaixa);
          break;
        }
      }
    }
  }

  if (caixaLeve == NULL) {
    printf("Caixa leve não encontrada.\n");
    wb_robot_cleanup();
    return 1;
  }

  int encontrou = 0, contador = 0, maxContador = 60 + rand() % 60;
  int recuando = 0, tempoRecuo = 0, ciclos = 0;
  int ciclosTravado = 0;
  double ultimaPosX = 0.0, ultimaPosZ = 0.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    ciclos++;

    const double *posCaixaAtual = wb_supervisor_node_get_position(caixaLeve);
    const double *posRobo = wb_supervisor_node_get_position(wb_supervisor_node_get_self());

    if (!posicaoSalva && ciclos >= TEMPO_INICIAL_IGNORADO) {
      posicaoInicialX = posCaixaAtual[0];
      posicaoInicialZ = posCaixaAtual[2];
      ultimaPosX = posRobo[0];
      ultimaPosZ = posRobo[2];
      posicaoSalva = 1;
      printf("Posição inicial da caixa leve salva: x=%.5f z=%.5f\n", posicaoInicialX, posicaoInicialZ);
    }

    if (posicaoSalva && !encontrou) {
      double dx = posCaixaAtual[0] - posicaoInicialX;
      double dz = posCaixaAtual[2] - posicaoInicialZ;
      double deslocamento = sqrt(dx * dx + dz * dz);
      //printf("Deslocamento da caixa leve: %.5f\n", deslocamento);
      if (deslocamento > LIMIAR_MOVIMENTO_CAIXA) {
        printf("Caixa leve foi empurrada! Iniciando rotação...\n");
        encontrou = 1;
      }
    }

    if (encontrou) {
      wb_motor_set_velocity(MotorEsquerdo, VELOCIDADE);
      wb_motor_set_velocity(MotorDireito, -VELOCIDADE);
      continue;
    }

    double movX = fabs(posRobo[0] - ultimaPosX);
    double movZ = fabs(posRobo[2] - ultimaPosZ);
    if (movX < LIMIAR_TRAVADO && movZ < LIMIAR_TRAVADO) {
      ciclosTravado++;
    } else {
      ciclosTravado = 0;
      ultimaPosX = posRobo[0];
      ultimaPosZ = posRobo[2];
    }

    if (ciclosTravado > CICLOS_TRAVADO) {
      printf("Robô travado! Recuando...\n");
      wb_motor_set_velocity(MotorEsquerdo, -VELOCIDADE);
      wb_motor_set_velocity(MotorDireito, -VELOCIDADE);
      wb_robot_step(10 * TIME_STEP);
      wb_motor_set_velocity(MotorEsquerdo, VELOCIDADE);
      wb_motor_set_velocity(MotorDireito, VELOCIDADE * 0.2);
      wb_robot_step(10 * TIME_STEP);
      ciclosTravado = 0;
      continue;
    }

    double ps0 = wb_distance_sensor_get_value(sensores[0]);
    double ps1 = wb_distance_sensor_get_value(sensores[1]);
    double ps6 = wb_distance_sensor_get_value(sensores[6]);
    double ps7 = wb_distance_sensor_get_value(sensores[7]);

    int caixa_proxima = 0;
    for (int i = 1; i <= QtddCaixas; i++) {
      char nomeCaixa[10];
      sprintf(nomeCaixa, "CAIXA%02d", i);
      WbNodeRef caixa = wb_supervisor_node_get_from_def(nomeCaixa);
      if (caixa != NULL) {
        const double *posCaixa = wb_supervisor_node_get_position(caixa);
        double dx = posCaixa[0] - posRobo[0];
        double dz = posCaixa[2] - posRobo[2];
        double dist = sqrt(dx * dx + dz * dz);
        if (dist < 0.09) {
          caixa_proxima = 1;
          break;
        }
      }
    }

    int sensores_parede =
      (ps0 > LIMIAR_PAREDE ? 1 : 0) +
      (ps1 > LIMIAR_PAREDE ? 1 : 0) +
      (ps6 > LIMIAR_PAREDE ? 1 : 0) +
      (ps7 > LIMIAR_PAREDE ? 1 : 0);
    
    // só considera parede se 2 ou mais sensores detectarem forte e não for caixa
    int parede = (sensores_parede >= 2) && !caixa_proxima;
    

    if (parede && !recuando) {
      printf("Parede detectada! Recuando...\n");
      recuando = 1;
      tempoRecuo = 15;
      wb_motor_set_velocity(MotorEsquerdo, -VELOCIDADE);
      wb_motor_set_velocity(MotorDireito, -VELOCIDADE);
    } else if (recuando) {
      tempoRecuo--;
      if (tempoRecuo <= 0) {
        recuando = 0;
        if (rand() % 2 == 0) {
          wb_motor_set_velocity(MotorEsquerdo, VELOCIDADE * 0.3);
          wb_motor_set_velocity(MotorDireito, VELOCIDADE);
        } else {
          wb_motor_set_velocity(MotorEsquerdo, VELOCIDADE);
          wb_motor_set_velocity(MotorDireito, VELOCIDADE * 0.3);
        }
      }
    } else {
      contador++;
      if (contador >= maxContador) {
        int dir = rand() % 3;
        switch (dir) {
          case 0:
            wb_motor_set_velocity(MotorEsquerdo, VELOCIDADE);
            wb_motor_set_velocity(MotorDireito, VELOCIDADE);
            break;
          case 1:
            wb_motor_set_velocity(MotorEsquerdo, VELOCIDADE * 0.5);
            wb_motor_set_velocity(MotorDireito, VELOCIDADE);
            break;
          case 2:
            wb_motor_set_velocity(MotorEsquerdo, VELOCIDADE);
            wb_motor_set_velocity(MotorDireito, VELOCIDADE * 0.5);
            break;
        }
        contador = 0;
        maxContador = 60 + rand() % 80;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}

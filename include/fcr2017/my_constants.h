/*		DEFINIÇÕES DAS CONSTANTES		*/

const float inf = 1.0/0.0;	// definição do infinito, usado em algumas comparações

const int deg = 3,	// graus por setor (3 graus)
	tamSet = deg*720/270,	// numero de medidas de cada setor (8 medidas)
	numSet = 270/deg;	// numero total de setores (90 setores)
	
const float valleyThr = 150.0;	// densidade maxima de um setor para que seja considerado parte de um vale

const int smax = 4;	// tamanho minimo (número de setores) para que um vale seja considerado valido

const float proxLimit1 = 1.0,	// limite de proximidade 1. ativa o desvio de obstaculos
	proxLimit2 = 0.5;	// limite de proximidade 2. faz a velocidade linear ser 0 durante o desvio

const double maxLinVel = 0.2,	// maxima velocidade linear
	maxAngVel = 0.2;	// maxima velocidade angular
	
const double cellSize = 0.5;	// tamanho de uma celula na grade de ocupacao

const Point posOffset (0, 0);   // posição inicial do robô
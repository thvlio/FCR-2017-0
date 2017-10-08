/*      DECLARAÇÕES DAS FUNÇÕES      */

// overloads e funções auxiliares
Point operator+ (const Point& augend, const Point& addend);
Point operator- (const Point& minuend, const Point& subtrahend);
double pdist (const Point& point1, const Point& point2);
double correctAngle (double theta);
int retrieveNode (std::vector<Node>& Graph, Point target);
void printRequestMsg (int currentNode);

// funções de detecção e desvio de obstáculos
std::vector<float> constructPod (const std::vector<float>& laser);
std::vector<float> smoothPod (const std::vector<float>& pod);
float findValley (const std::vector<float>& spod, Point robotPos, Point goal);
Velocity evadeObject (const std::vector<float>& laser, Point robotPos, Point target, const std::vector<float>& closestObs);
std::vector<float> findClosestObstacle (const std::vector<float>& laser);

// funções de planejamento e execução de caminhos
void constructGraph (std::vector<Node>& Graph);
std::vector<Node> searchGraph (std::vector<Node>& Graph, int sourceIndex, int targetIndex);
void constructPathFromFile (std::vector<Point>& path, const std::vector<Node>& graph, int currentNode);
void constructPathFromPosition (std::vector<Point>& path, const std::vector<Node>& sequence, Point finalPoint);
Velocity followGoal (Point robotPos, Point goal);
Velocity finalAdjustment (Point robotPos, Point targetPoint, int& spin);
Velocity manageVel (Velocity evadeVel, Velocity followVel, std::vector<float> closestObs);

// funções de mapeamento
void printGridToFile (const Grid& grid, int nodeNum);
void constructMap (std::vector<Grid>& map);
void constructOccupancyGrid (Grid& grid, const std::vector<float>& laser, Point robotPos, int& robotCelli, int& robotCellj);
void printOccupancyGrid (const matrix_t& matrix, int robotCelli, int robotCellj, int currentNode);
void writeMatrixOnImage (const matrix_t& matrix, int nodeNum, int scale);
Velocity lookAround (Point robotPos, Point goal, int& spin);











/*		OVERLOADS E FUNÇÕES AUXILIARES		*/

/*	operator+: soma de dois pontos.	*/
Point operator+ (const Point& augend, const Point& addend) {
	Point sum (augend.x + addend.x, augend.y + addend.y);
	return sum;
}

/*	operator-: subtração de dois pontos.	*/
Point operator- (const Point& minuend, const Point& subtrahend) {
	Point difference (minuend.x - subtrahend.x, minuend.y - subtrahend.y);
	return difference;
}

/*	pdist: calcula a distância entre dois pontos.	*/
double pdist (const Point& point1, const Point& point2) {
	return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
}

/*	correctAngle: traz um ângulo para a faixa de -180 a 180 graus.	*/
double correctAngle (double theta) {
	if (theta > 180.0)
		return (theta - 360.0);
	else if (theta < -180.0)
		return (theta + 360.0);
	else return theta;
}

/*	retrieveNode: dado um ponto, determina a qual nó pertence o ponto. Se não pertencer a nenhum, retorna zero.	*/
int retrieveNode (std::vector<Node>& Graph, Point target) {
	
	int i, found = 0;
	
	for (i = 0; (i < Graph.size()) && (!found); ++i)
		if (Graph[i].isInside(target))
			found = 1;
	
	if (!found)
		return 0;
	else
		return i;
}











/*		FUNÇÕES DE DETECÇÃO E DESVIO DE OBSTÁCULOS		*/

/*	constructPod: constrói e retorna um vetor com a densidade polar de objetos de cada setor.	*/
std::vector<float> constructPod (const std::vector<float>& laser) {
	
	std::vector<float> pod (numSet, 0);	// primeiro, inicializa-se o vetor com 0
	
	// a e b são duas constantes usadas pra calcular a densidade polar de um setor. c e m são duas variáveis na fórmula. as equações podem ser conferidas no artigo original que explica o método VFH, com algumas adaptações explicadas abaixo
	float a = 3.0,
		b = 0.1,
		c,
		m;
	
	// aqui eu calculo a densidade de objetos em cada setor. o laço calcula m para cada medida e soma os valores dentro de um só setor para encontrar a densidade. uma adaptação feita para o código foi o cálculo de c, que deveria ser um valor probabilístico mas aqui eu calculei os valores com base nas medidas do sensor laser na posições i, i+1 e i-1
	for (int j = 0; j < numSet; ++j) {
		for (int i = j * tamSet; i < (j+1) * tamSet; ++i) {
			if (laser[i] != inf) {
				c = 1;
				if ((i != 0) && (std::abs(laser[i] - laser[i-1]) < 0.2))
					c+=2;
				if ((i != 719) && (std::abs(laser[i] - laser[i+1]) < 0.2))
					c+=2;
				m = c*c*(a-b*laser[i]);
			} else
				m = 0;
			pod[j] += m;
		}
	}
	
	return pod;
}

/* 	smoothPod: aplica uma função de suavização ao vetor de densidades e retorna o vetor resultante. A função de suavização foi baseada na função descrita no artigo original sobre VFH, com algumas adaptações.	*/
std::vector<float> smoothPod (const std::vector<float>& pod) {
	
	std::vector<float> spod (numSet, 0);
	
	for (int i = 0; i < numSet; ++i) {
		for (int j = -5; j < 6; ++j) {
			if (((i+j) > -1) && ((i+j) < numSet)) {
				spod[i] += ((6 - std::abs(j)) * pod[i+j]);
			}
		}
		spod[i] /= 36;
	}
	
	return spod;
}

/*	findValley: acha o vale com direção mais próxima à do objetivo atual e retorna a direção.	*/
float findValley (const std::vector<float>& spod, Point robotPos, Point goal) {
	
	float closestToCurGoal = inf;	// closestToCurGoal é o ângulo mais proximo da orientacao sendo seguida atualmente
	double minDiff = inf;	// menor diferença entre o ângulo de um vale e o ângulo da reta que liga o robô ao alvo
	
	for (int i = 0; i < numSet; ++i) {
	
		if (spod[i] < valleyThr) {
		
			// quando um começo de vale é encontrado, o tamanho do vale e seu ângulo médio são determinados
			int cursize = 0;
			for (int j = i; (j < numSet) && (spod[j] < valleyThr); ++j)
				cursize++;
			float avgAngle = (i+cursize/2.0) * deg - 135.0;
			
			// e a diferença entre o ângulo do vale e da reta que liga o robô ao seu alvo é calculada. o objetivo é que a diferença calculada seja zero, então o vale selecionado será o vale com a orientação mais próxima da orientação da reta sendo seguida
			double diff = avgAngle - std::atan2 (goal.y-robotPos.y, goal.x-robotPos.x) * 180.0/M_PI;
			diff = correctAngle (diff);
			if ((cursize >= smax) && (std::abs(diff) < std::abs(minDiff))) {
				closestToCurGoal = avgAngle;
				minDiff = diff;
			}	
			
			i = i + cursize - 1;	// a busca por novos vales começa do final do vale atual
			
		}
		
	}
	
	return closestToCurGoal;
}

/* 	evadeObject: determina um comando de velocidade para que desvie de um obstáculo e siga um vale.	*/
Velocity evadeObject (const std::vector<float>& laser, Point robotPos, Point target, const std::vector<float>& closestObs) {
	
	Velocity robotVel;
	
	// abaixo o histograma é construído um vale é encontrado
	std::vector<float> pod = constructPod (laser);
	std::vector<float> spod = smoothPod (pod);
	float valleyDir = findValley (spod, robotPos, target);
	
	// o robô gira até que esteja alinhado com a direção do vale, sempre com alguma velocidade linear
	robotVel.lin = maxLinVel/2;
	if (valleyDir > 2.0)
		robotVel.ang = maxAngVel;
	else if (valleyDir < -2.0)
		robotVel.ang = -maxAngVel;
	else
		robotVel.ang = 0.0;
	
	// se o robô detectar algo muito perto, ele traz sua velocidade linear para zero
	if (closestObs[0] < proxLimit2)
		if (std::abs(valleyDir) > 2.0)
			robotVel.lin = 0.0;
	
	return robotVel;
	
}

/*	findClosestObstacle: procura a menor distância no vetor das medidas do sensor laser.	*/
std::vector<float> findClosestObstacle (const std::vector<float>& laser) {
	
	// o vetor retornado é um ponto em coordenadas polares. uma consequência de eu determinar que o ângulo começa com inf é que o robô sempre gira no sentido anti-horário para procurar um vale, e nunca no sentido horário. depois que ele acha um vale, ele gira no sentido de menos esforço
	std::vector<float> polarPosObstacle (2, inf);
	
	for (int i = 0; i < laser.size(); ++i) {
		if ((laser[i] != inf) && (laser[i] < polarPosObstacle[0])) {
			polarPosObstacle[0] = laser[i];
			polarPosObstacle[1] = i * 270.0/720.0 - 135.0;	// o índice é transformado num ângulo
		}
	}
	
	return polarPosObstacle;	
}











/*		FUNÇÕES DE PLANEJAMENTO E EXECUÇÃO DE CAMINHOS 		*/

/*	constructGraph: contrói um grafo a partir do mapa do CIC,	*/
void constructGraph (std::vector<Node>& Graph) {
	
	// primeiro, o arquivo graph.txt é aberto para a leitura. veja que o comando rosrun deve ser executado na pasta onde se encontra o arquivo texto. atualmente, a pasta é a mesma dos arquivos-fonte do pacote
	std::ifstream graphFile ("graph.txt", std::ios::in);
	
	if (graphFile.is_open()) {
		
		std::cout << "[GRAPH FILE LOADED SUCCESSFULLY]\n";
		
		int numNodes;	// número total de nós do grafo
	
		graphFile >> numNodes;
		Graph.reserve (numNodes);
		
		for (int i = 0; i < numNodes; ++i) {
			
			double x1, x2, y1, y2;	// limites da região de um nó
			graphFile >> x1 >> x2 >> y1 >> y2;
			Node tempNode (x1, x2, y1, y2, i);	// um nó temporário é criado com index i
			
			int numAdj;	// número de nós adjacentes ao nó atual
			graphFile >> numAdj;
			
			// abaixo os vizinhos são lidos e arcos são acrescentados ao nó
			for (int j = 0; j < numAdj; ++j) {
				int adj;	// nó a ser ligado ao nó atual
				double weight;	// peso do arco
				graphFile >> adj >> weight;
				Edge newEdge (&Graph[adj-1], weight);
				tempNode.edges.push_back (newEdge);
			}
			
			Graph.push_back (tempNode); // por fim, o nó temporário é acrescentado ao vetor Graph
		}
		
		graphFile.close();
		
	} else
		std::cout << "[ERROR: GRAPH TEXT FILE NOT FOUND]\n";
		
}

/*	searchGraph: procura o caminho de menor tamanho que liga os dois nós indicados. O algoritmo da função foi retirada do artigo da Wikipedia sobre o algoritmo de Dijkstra e ele foi implementado quase que ao pé da letra, com os mesmos nomes das variáveis.	*/
std::vector<Node> searchGraph (std::vector<Node>& Graph, int sourceIndex, int targetIndex) {
	
	double alt;	// alt é a distância de um caminho alternativo
	
	// o vetor set armazenará o conjunto de nós não visitados por meio de ponteiros
	std::vector<Node*> set;	
	
	Node* u = NULL;	// u e v são ponteiros de nós auxiliares
	Node* v = NULL;
	
	// dist é um vetor que armazenará as distâncias dos nós do grafo até a origem (source) fornecida. os índices para se acessar o vetor são os mesmos dos índices para se acessar o vetor de nós Graph
	std::vector<double> dist (Graph.size(), inf);
	
	// prev é um vetor que será preenchido conforme o caminho mais curto for criado. ele armazena a sequência de nós de forma que, se u é um nó e v é o nó anterior a ele no caminho determinado, então prev[u] = v. veja que prev[u] = v não é uma operação aceita, já que u é precisa ser um índice inteiro e não um ponteiro. conforme mencionado anteriormente, o membro index dos nós vai facilitar esta notação, já que o equivalente seria prev[u->index] = v;
	std::vector<Node*> prev (Graph.size(), NULL);
	
	// primeiro, os nós são adicionados ao conjunto de nós não visitados
	for (int i = 0; i < Graph.size(); ++i)
		set.push_back(&Graph[i]);
	
	dist[sourceIndex] = 0.0;	// a distância da origem até a origem é zero
	
	while ((!set.empty()) && (u != &Graph[targetIndex])) {
	
		// primeiro, procura-se o nó no conjunto de nós não visitados mais próximo do nó atual
		u = set[0];
		int j = 0;
		for (int i = 0; i < set.size(); ++i) {
			if (dist[set[i]->index] < dist[u->index]) {
				u = set[i];	// u aponta para o nó selecionado
				j = i;	// j é a posição de u em set
			}
		}
		
		if (u != &Graph[targetIndex]) { 
			set.erase(set.begin()+j);	// depois que um nó for selecionado, ele é removido de set, já que está sendo visitado
		
			// agora, o algoritmo procura novos caminhos que possam ser menores que o caminho atual
			for (int i = 0; i < u->edges.size(); ++i) {
				v = u->edges[i].pointTo;	// v recebe um dos vizinhos de u
				alt = dist[u->index] + u->edges[i].weight;	// um novo caminho é calculado indo de u para v
				if (alt < dist[v->index]) {	// se este caminho for menor do que o caminho atual da origem até v
				   	dist[v->index] = alt;	// este caminho se torna o novo menor caminho
					prev[v->index] = u;		// e u é definido no vetor prev como sendo o nó anterior a v
				}
			}
		}
	}
	
	// por fim, a sequência de nós armazena em prev é traduzida num outro vetor de nós em que nós estão dispostos na ordem em que devem ser percorridos
	std::vector<Node> seq;
	while(prev[u->index] != NULL) {
		seq.insert(seq.begin(), *u);
		u = prev[u->index];
	}
	seq.insert(seq.begin(), *u);
	
	return seq;
	
}

/*	constructPathFromFile: constrói e retorna um vetor com vários pontos que servirão de alvo para o robô. A sequência de nós é lida de um arquivo texto.	*/
void constructPathFromFile (std::vector<Point>& path, const std::vector<Node>& graph, int currentNode) {
	
	std::ifstream textFile ("sequence.txt", std::ios::in);
	
	if (textFile.is_open()) {
		std::cout << "[NODE SEQUENCE FILE LOADED SUCCESSFULLY]\n";
		int seqSize;	// tamanho da sequência de nós
		textFile >> seqSize;
		for (int i = 0; i < seqSize; ++i) {
			int node;	// nó da sequência lido do arquivo
			textFile >> node;
			Point tempPoint (graph[node-1].x, graph[node-1].y);	
			path.push_back(tempPoint);	// o centro do nó é acrescentado ao caminho
		}
	} else
		std::cout << "[ERROR: SEQUENCE TEXT FILE NOT FOUND]\n";

	
}

/*	constructPathFromPosition: constrói o vetor de pontos a partir de uma sequência dada pelo algoritmo de Dijkstra.	*/
void constructPathFromPosition (std::vector<Point>& path, const std::vector<Node>& sequence, Point finalPoint) {
	
	path.reserve(sequence.size());
	
	for (int i = 0; i < sequence.size(); ++i) {
		Point tempGoal (sequence[i].x, sequence[i].y);
		path.push_back(tempGoal);
	}
	
	path.push_back(finalPoint);
	
}

/*	followGoal: retorna um comando de velocidade que faz o robô seguir em direção a um alvo.	*/
Velocity followGoal (Point robotPos, Point goal) {
	
	Velocity robotVel;
	
	// calcula a diferença de orientação do robô com relação à reta que liga o robô ao alvo. por conta da descontinuidade na transição de pi para -pi, a diferença de orientação deve ser corrigida se for maior que 180 ou menor que -180
	double diff = robotPos.yaw - std::atan2 (goal.y-robotPos.y, goal.x-robotPos.x) * 180.0/M_PI;
	diff = correctAngle (diff);
	
	double limit = 35.0;	// angulo a partir do qual a vel linear aumenta e a vel angular diminui
	
	if (std::abs(diff) < limit) {
		robotVel.lin = maxLinVel * (1 - std::abs(diff)/limit);
		robotVel.ang = maxAngVel * (-diff)/limit;
	} else {	// aqui ficam definidas as velocidades padrão para ângulos maiores que o limite definido acima
		robotVel.lin = 0.0;
		if (diff < 0.0)
			robotVel.ang = maxAngVel;
		else
			robotVel.ang = -maxAngVel;
	}
	
	return robotVel;
		
}

/*	finalAdjustment: função que gira o robô para que ele se posicione de acordo com a orientação dada pelo usuário. O esqueleto da função é parecido com o de followGoal.	*/
Velocity finalAdjustment (Point robotPos, Point targetPoint, int& spin) {
	
	Velocity robotVel;
	robotVel.lin = 0.0;
	
	double diff = robotPos.yaw - targetPoint.yaw;
	
	diff = correctAngle (diff);
	
	double limit = 35.0;
	
	if (std::abs(diff) < limit)
		robotVel.ang = maxAngVel * (-diff)/limit;
	else {
		robotVel.ang = maxAngVel;
		if (diff > 0.0)
			robotVel.ang = -robotVel.ang;
	}
	
	if (std::abs(diff) < 5.0)
		spin = 0;
		
	return robotVel;
	
}

/*	manageVel: determina a junção dos dois comandos de velocidade, evadeVel e followVel. Os comportamentos são escolhidos ou mesclados de forma linear com base na distância mínima do robô a um obstáculo.	*/
Velocity manageVel (Velocity evadeVel, Velocity followVel, std::vector<float> closestObs) {

	if (closestObs[0] < proxLimit2)
		return evadeVel;
		
	else if (closestObs[0] < proxLimit1) {
		Velocity tempVel;
		tempVel.lin = evadeVel.lin + (followVel.lin - evadeVel.lin) * (closestObs[0] - proxLimit2)/(proxLimit1 - proxLimit2);
		tempVel.ang = evadeVel.ang + (followVel.ang - evadeVel.ang) * (closestObs[0] - proxLimit2)/(proxLimit1 - proxLimit2);
		return tempVel;
		
	} else
		return followVel;
	
}











/*		FUNÇÕES DE MAPEAMENTO		*/

/*	printGridToFile: escreve a grade num arquivo texto.	*/
void printGridToFile (const Grid& grid, int nodeNum) {
	
	std::stringstream fileName;	// aqui eu tento uniformizar os nomes dos arquivo criados
	fileName << "grids/node" << nodeNum << ".txt";
	std::ofstream textFile (fileName.str().c_str(), std::ios::out | std::ios::trunc); // o arquivo sobrescreve os arquivos anteriores
	
	// aqui eu defino a precisão dos números impressos e salvo a precisão atual
	std::streamsize ss = textFile.precision();
	textFile << std::setprecision (3);
	textFile << std::fixed;
	
	textFile << grid.width << " " << grid.height << "\n";
	for (int j = grid.matrix[0].size()-1; j > -1; --j) {
		for (int i = 0; i < grid.matrix.size(); ++i)
			textFile << grid.matrix[i][j] << " ";
		textFile << "\n";
	}
	
	// aqui eu volto a precisão ao normal
	textFile.unsetf (std::ios::floatfield);
	textFile << std::setprecision(ss);
	
	textFile.close();
	
}

/*	constructMap: lê as regiões dos nós de um arquivo texto.	*/
void constructMap (std::vector<Grid>& map) {
	
	// primeiro, as informações sobre os nós são lidas do arquivo nodes.txt. talvez eu devesse ter dado um jeito de ler do arquivo graph.txt, mas não rolou ¯\_(ツ)_/¯
	std::ifstream textFile ("nodes.txt", std::ios::in);
	
	if (textFile.is_open()) {
		
		std::cout << "[NODE INFORMATION FILE LOADED SUCCESSFULLY]\n";
		
		int numNodes;	// número total de nós do mapa
		textFile >> numNodes;
		map.reserve (numNodes);
		
		for (int i = 0; i < numNodes; ++i) {
			// para cada nó um grid é criado e anexado ao mapa
			int width, height;
			double x, y;
			textFile >> x >> y >> width >> height;
			Grid tempGrid (x, y, width, height, i+1);
			map.push_back (tempGrid);
			// os arquivos de texto e imagem são inicializados
			printGridToFile (tempGrid, i+1);
			writeMatrixOnImage (tempGrid.matrix, i+1, 50);
		}
		
	} else
		std::cout << "[ERROR: NODE INFORMATION FILE NOT FOUND]\n";
	
}

/*	constructOccupancyGrid: contrói a grade de ocupação. O algoritmo itera na matriz para construir cada célula individualmente. De um modo geral, o robô determina, para cada célula, qual é a posição no vetor laser que ele tem que investigar pra saber se a célula está ou não está ocupada. Com esse valor, ele procura nesse índice e em alguns vizinhos (só pra garantir) se a medida, transformada para coordenadas retangulares, está dentro da célula.	*/
void constructOccupancyGrid (Grid& grid, const std::vector<float>& laser, Point robotPos, int& robotCelli, int& robotCellj) {
	
	for (int j = 0; j < grid.matrix[0].size(); ++j) {
		
		for (int i = 0; i < grid.matrix.size(); ++i) {
			
			// é melhor tratar cada ponto como um vetor pra entender melhor. aqui eu somo a posição do hokuyo à posição do robô, já que a distância está sendo medida com relação ao hokuyo, não ao robô. descobri essa distância no arquivo .xacro do robô. a origem do rangefinder, se eu fiz as contas certo no arquivo, é <0.165 0 0.285>. isso é exato para o robô da simulação, e imagino que seja uma boa estimativa para o robô real
			Point hokuyoPos (robotPos.x + 0.165*std::cos (robotPos.yaw*M_PI/180.0),
							robotPos.y + 0.165*std::sin (robotPos.yaw*M_PI/180.0));
            
			// aqui eu defino o centro da célula atual a partir dos dados da grade, como center, width e height
			Point cellCenter (grid.center.x + (i-grid.width/2) * cellSize, grid.center.y + (j-grid.height/2) * cellSize);

			Point cellRelHokuyo = cellCenter - hokuyoPos;	// posição da célula com relação ao hokuyo

			// com isto, é possível calcular qual é o ângulo da célula com relação ao hokuyo
			double angCellCenterRelToHokuyo = std::atan2 (cellRelHokuyo.y, cellRelHokuyo.x) * 180.0/M_PI - robotPos.yaw;
			angCellCenterRelToHokuyo = correctAngle (angCellCenterRelToHokuyo);

			int k = (angCellCenterRelToHokuyo + 135.0) * 8.0/3.0;	// índice no vetor que corresponde a esse ângulo

			// aqui eu encontro a distância do centro da célula com relacao ao hokuyo. isso vai servir pra avaliar se a célula deve receber zero, indicando que a medida do laser está depois da célula. isso aqui eu acho que é maior vantagem de analisar cada célula da matriz ao invés de montar a grade a partir do vetor de leituras do laser. aqui eu determino facilmente quando uma célula é zero com base nessa distância
			double distCellCenterToHokuyo = std::sqrt(std::pow(cellRelHokuyo.y, 2) + std::pow(cellRelHokuyo.x, 2));
			
			// analisando o laser[k] e alguns dos seus vizinhos
			for (int l = k-4; l < k+5; ++l) {
				
				if (l >= 0 && l <= 719) {
					
					// transforma a leitura do sensor num ponto com relação ao hokuyo e depois num ponto global
					Point measureRelHokuyo (laser[l]*std::cos ((l*3.0/8.0-135.0+robotPos.yaw)*M_PI/180.0),
											laser[l]*std::sin ((l*3.0/8.0-135.0+robotPos.yaw)*M_PI/180.0));
					Point measureGlobal = measureRelHokuyo + hokuyoPos;
					
					// aqui vem o porque de eu manter um contador de número de iterações. isso me permite usar uma média ponderada móvel pra simular as probabilidades de a célula estar ocupada ou não. de certa forma, esse método é um tipo de convolução e pode ser considerado um filtro. as fórmulas foram retiradas da Wikipedia e adaptadas para o caso de um cálculo iterativo. não é nenhum filtro de Bayes mas acho que deu uma estimativa legal ¯\_(ツ)_/¯
					
					int n = grid.iterNum;	// pra facilitar a notação
					
					// aqui o código verifica se measure global está dentro da célula. a média ponderada móvel funciona mais ou menos da seguinte maneira. primeiro, a medida anterior é multiplicada pelo peso total da iteração anterior. o peso total de uma medida n é dado pelo somatório dos n primeiros inteiros. esse somatório tem um valor conhecido, dado por 1 + 2 + ... + n-1 + n = n*(n+1)/2. para n-1, temos (n-1)*(n)/2. o que esse trecho faz é multiplicar o valor anterior da medida (grid.matrix[i][j]) pelo peso total anterior ((n-1)*(n)/2), somar isso a 1 * n, que é a medida atual (1) multiplicada pelo seu peso (n), e depois dividir tudo pelo novo peso total ((n-1)*(n)/2).
					if ((std::abs(measureGlobal.x-cellCenter.x) < cellSize/2) && (std::abs(measureGlobal.y-cellCenter.y) < cellSize/2))
						grid.matrix[i][j] = (grid.matrix[i][j] * ((n-1)*n/2) + n*1)/(n*(n+1)/2);
					
					// agora, se a medida não está dentro da célula e a medida é maior do que a distância do hokuyo até o centro da célula, então a célula está vazia. veja que isto é uma simplificação, e só seria verdade sempre se a medida [k] passasse exatamente pelo centro da célula. eu só uso a medida k nessa vericação, já que é o índice que, teoricamente, pass mais perto do centro. neste caso, ao invés de somar 1*n ao total, soma-se 0*n, já que a célula está vazia
					else if ((l == k) && (k >= 0 && k <= 719) && (laser[k] > distCellCenterToHokuyo))
						grid.matrix[i][j] = (grid.matrix[i][j] * ((n-1)*n/2) + n*0)/(n*(n+1)/2);
						
				}
				
			}
			
			// esse trecho determina qual é a posição do robô na matriz
			if ((std::abs(hokuyoPos.x-cellCenter.x) < cellSize/2) && (std::abs(hokuyoPos.y-cellCenter.y) < cellSize/2)) {
				robotCelli = i;
				robotCellj = j;
				grid.matrix[i][j] = 0;	// a posição do robô sempre está livre de obstáculos
			}
			
		}
		
	}
	
	grid.iterNum++;
	
}

/*	printOccupancyGrid: mostra no terminal a grade de ocupação com color coding e com duas casas decimais fixas de precisão.	*/
void printOccupancyGrid (const matrix_t& matrix, int robotCelli, int robotCellj, int currentNode) {
	
	// formata o output para duas casas decimais fixas ou só uma no caso de grades muito largas
	std::streamsize ss = std::cout.precision();
	if (currentNode == 2 || currentNode == 5)
		std::cout << std::setprecision(1);
	else
		std::cout << std::setprecision(2);
	std::cout << std::fixed;
	
	// configura algumas cores
	std::string escRed = "\033[31;1m",
		escGreen = "\033[32;1m",
		escYellow = "\033[33;1m",
		escBlue = "\033[34;1m",
		escReset = "\033[0m";
	
	std::cout << "Occupancy Grid:\n";
	for (int j = matrix[0].size()-1; j > -1; --j) {
		for (int i = 0; i < matrix.size(); ++i) {
			
			// os limites para as probabilidades foram definidos arbitrariamente
			if (i == robotCelli && j == robotCellj)
				std::cout << escBlue;	// colore a posição do robo de azul
			else if (matrix[i][j] >= 0.7)
				std::cout << escRed;	// colore obstaculos de vermelho
			else if (matrix[i][j] >= 0.2)
				std::cout << escYellow;	// colore celulas incertas de amarelo
			else if (matrix[i][j] >= 0.0)
				std::cout << escGreen;	// colore espaços livres de verde
			
			std::cout << matrix[i][j] << " ";
			std::cout << escReset; // reseta as cores do output
			
		}
		std::cout << "\n";
	}
	std::cout << "\n";
	
	// volta a formatação do output para o padrão
	std::cout.unsetf(std::ios::floatfield);
	std::cout << std::setprecision(ss);
	
}

/*	writeMatrixOnImage: escreve uma matrix num arquivo bitmap.	*/
void writeMatrixOnImage (const matrix_t& matrix, int nodeNum, int scale) {
    
	std::stringstream imageName;	// a escrita das imagens é parecida com a dos arquivos texto
	imageName << "bitmaps/image" << nodeNum << ".bmp";
	std::ofstream image (imageName.str().c_str(), std::ios::binary | std::ios::out | std::ios::trunc);
	
    // estes são os valores de width e height da matriz aumentada. a matriz precisa ser aumentada na impressão porque
	// não rola de associar cada célula a só um pixel, né :)
    int width = matrix.size()*scale,
        height = matrix[0].size()*scale;
    
    // tamanho do array de pixels, em bytes. a fórmula, retirada da Wikipedia, é floor([BitsPerPixel*ImageWidth+31]/32)*4*|ImageHeight|
    int pixArrSize = (24*width+31)/32 * 4 * height;
    
	// tamanho total do arquivo. basta somar pixArrSize com os cabeçalhos, de 14 e 40 bytes (14 + 40 = 54 = 0x36)
    int fileSize = pixArrSize + 0x36;	
    
    // inicialização do fileheader do arquivo. as informações devem ser no formato litte-endian
    BitmapFileHeader fileHeader = { // as mascaras e os shifts sao pra converter de big endian pra little endian
        {0x42, 0x4D},   // assinatura da imagem, sempre igual a BM para bitmap
        {fileSize&0xFF, (fileSize&0xFF00)>>8, (fileSize&0xFF0000)>>16, (fileSize&0xFF000000)>>24},   // tamanho total do arquivo
        {0x00, 0x00},   // nao importa, depende da aplicacao
        {0x00, 0x00},   // nao importa, depende da aplicacao
        {0x36, 0x00, 0x00, 0x00}    // tamanho dos cabecalhos juntos, sempre 54 bytes no meu caso
    };
    
    // inicialização do infoheader do arquivo
    BitmapInfoHeader infoHeader = {
        {0x28, 0x00, 0x00, 0x00},   // tamanho do infoheader, sempre 40 bytes no meu caso
        {width&0xFF, (width&0xFF00)>>8, (width&0xFF0000)>>16, (width&0xFF000000)>>24},   // largura em pixels da imagem
        {height&0xFF, (height&0xFF00)>>8, (height&0xFF0000)>>16, (height&0xFF000000)>>24},   // altura em pixel da imagem
        {0x01, 0x00},   // nao importa, no meu caso sempre so tem um plano de cores
        {0x18, 0x00},   // bits por pixel, no meu caso escolhi 24
        {0x00, 0x00, 0x00, 0x00},   // nao importa, metodo de compressao padrao
        {pixArrSize&0xFF, (pixArrSize&0xFF00)>>8, (pixArrSize&0xFF0000)>>16, (pixArrSize&0xFF000000)>>24},   // tamanho de pixelarray
        {0x13, 0x0B, 0x00, 0x00},   // resolucao horizontal, nao importa muito (72 dpi)
        {0x13, 0x0B, 0x00, 0x00},   // resolucao vertical, nao importa muito (72 dpi)
        {0x00, 0x00, 0x00, 0x00},   // nao importa, todas as cores estao na paleta
        {0x00, 0x00, 0x00, 0x00},   // nao importa, todas as cores sao importantes
    };
    
    // abaixo os headers são escritos no arquivo. ah, algumas coisas que esqueci de dizer. talvez minha escolha dos tipos na hora de criar as estruturas tenha sido uma escolha infeliz, mas eu preferi deixar tudo homogêneo e trabalhar só com unsigned char pra eu não me perder. no fim das contas, bastava esse casting pra char* que ele imprimia tudo certinho no arquivo. é a melhor solução? não sei, mas é o que temos pra hoje ¯\_(ツ)_/¯
    image.write ((char*) &fileHeader, sizeof(BitmapFileHeader));
    image.write ((char*) &infoHeader, sizeof(BitmapInfoHeader));
    
    // aqui vem a parte legal do cógido. ou não, né rsrs ¯\_(ツ)_/¯
    // o basicao que o codigo faz lá embaixo é o seguinte
    // 
    // int n = 0;
    // for (int j = 0; j < height; ++j) {
    //     int m = 0;
    //     for (int i = 0; i < width; ++i) {
    // 
    //          // essa parte determina um tom de cinza com base no dado da célula.
    //          // o tom é designado a um pixel e esse pixel é escrito no arquivo.
    //          unsigned char pixel [3],
    //             shade = 0xFF * (1 - matrix[m][n]);
    //          assignColorToPixel(pixel, shade, shade, shade);
    //          image.write ((char*) pixel, 3*sizeof(unsigned char));
    // 
    //          // com isso aqui eu consigo definir uma escala pra imagem. imprimir um pixel por celula nao da ne rsrs.
    //          // quando i é um múltiplo da escala, m recebe mais 1 e a gente passa pra proxima celula
    //          if (i % scale == (scale-1))
    //             ++m;
    //     }
    //     // mesma coisa aqui
    //     if (j % scale == (scale-1))
    //         ++n;
    // }
    // 
    // minha primeira ideia foi usar k % scale == 0, mas isso acaba deslocando a matriz e não definindo
    // corretamente os elementos das bordas da matrix. estou usando k % scale == scale-1 porque foi esse padrão
    // que vi que deveria ser usado enquanto eu testava o programa. pra te falar a verdade, só entendi o porque
    // disso depois que fiquei batendo cabeça com a matemática da divisão e do resto e etc.
    // 
    // de qualquer forma essa é a ideia. acontecem poucas coisas a mais no codigo abaixo. uma é que a matrix é impressa
    // de cabeça pra baixo, entao os indices sao tratados de forma diferente. onde ele começava em 0, bota que ele começa
    // height-1 e vai até 0. além disso, naquele calculo do resto, basta ver que se j começa de height-1 e vai ate 0
    // entao podemos dizer que k = height-1-j e depois j = k. outra coisa diferente é que m e n sao inicializados com i e j
    // ali na declaracao do for. por fim, temos o padding.
    // 
	// EDIT: não precisei escrever a matriz de cabeça pra baixo. ela já estava de cabeça pra baixo na memória ¯\_(ツ)_/¯
	//
    // por algum motivo que eu não entendi uma linha de pixels sempre tem que ser múltipla de 4 bytes no tamanho. sempre.
    // por isso, sempre que você imprimir sua linha e ela nao for multipla de 4, voce precisa botar uns bytes so pra completar
    // a linha. o trecho abaixo faz essa verificacao, ja que meu pixels sao de 3 bytes e pode rolar de precisar do padding.
    // 
    // int paddingSize = 4 - (width * 3) % 4;
    // if (paddingSize != 4) {
    //     unsigned char *padding = new unsigned char [paddingSize];
    //     for (int k = 0; k < paddingSize; ++k)
    //         padding[k] = 0x00;
    //     image.write ((char*) padding, (paddingSize)*sizeof(unsigned char));
    //     delete[] padding;
    // }
    // 
    // pronto. a ideia do algoritmo eh essa. acredito que ele funcione com qualquer escala, desde scale=1 até um número alto
    // o suficiente pra estourar a memória. por favor, não bote números negativos nem zero porque eu não sei o que vai rolar. :)
    
	for (int j = 0, n = 0; j < height; ++j) {
        for (int i = 0, m = 0; i < width; ++i) {
            unsigned char pixel [3],
                shade = 0xFF * (1 - matrix[m][n]);
			pixel[0] = shade;
			pixel[1] = shade;
			pixel[2] = shade;
            image.write ((char*) pixel, 3*sizeof(unsigned char));
            if (i % scale == (scale-1))
                ++m;
        }
		if (j % scale == (scale-1))
			++n;
        int paddingSize = 4 - (width * 3) % 4;
        if (paddingSize != 4) {
            unsigned char *padding = new unsigned char [paddingSize];
            for (int k = 0; k < paddingSize; ++k)
                padding[k] = 0x00;
            image.write ((char*) padding, (paddingSize)*sizeof(unsigned char));
            delete[] padding;
        }
    }
    
	image.close();
	
}

/*	lookAround: faz o robô girar em torno de si mesmo quando chega no centro de um nó.	*/
Velocity lookAround (Point robotPos, Point goal, int& spin) {
	
	Velocity robotVel;
	robotVel.lin = 0.0;
	
	// calcula a diferença entre a orientação do robô e a reta que liga o robô ao alvo somada de 180 graus
	double diff = robotPos.yaw - std::atan2 (goal.y-robotPos.y, goal.x-robotPos.x) * 180.0/M_PI + 180.0;
	diff = correctAngle (diff);
	
	double limit = 35.0;	// angulo a partir do qual a vel angular diminui
	
	// o controle é o mesmo da função followGoal
	if (std::abs(diff) < limit)
		robotVel.ang = maxAngVel * (-diff)/limit;
	else {
		if (diff < 0.0)
			robotVel.ang = maxAngVel;
		else
			robotVel.ang = -maxAngVel;
	}
	
	// depois de girar 165 graus (180.0 - 15.0), o robô não precisa mais girar e seta spin como zero
	if (std::abs(diff) < 15.0)
		spin = 0;
	
	return robotVel;
	
}

/*  printRequestMsg: indica ao usuário que ele pode inserir um comando de posição no terminal.  */
void printRequestMsg (int currentNode) {
	
	std::cout << "O mapeamento foi finalizado. Abaixo temos os nós válidos no prédio do CIC:\n";
	std::cout << ".\t.\t.\t.\t.\t.\t.\n";
	std::cout << ".\t\t.\t\t4\t\t.\n";
	std::cout << ".\t.\t1\t2\t3\t5\t.\n\n";
	std::cout << "O robô está no nó " << currentNode << ".\n\n";
	std::cout << "Entre com o ponto de chegada e a orientação final do robô [três números com espaço entre si]. O ponto (x, y) deve pertencer aos corredores do CIC mapeados anteriormente e a orientação deve ser um número entre -180.0 e 180.0.\n";
	std::cout << "[AGUARDANDO MENSAGEM]\n";
	
}
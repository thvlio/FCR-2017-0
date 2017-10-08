#include "fcr2017/obstacle_avoidance.h"
#include <fcr2017/my_constants.h> // 
#include <fcr2017/my_functions.h> //

ObstacleAvoidance::ObstacleAvoidance (ros::NodeHandle nh): nh_(nh) {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("cmd_vel", 10);
	grid_pub_ = nh_.advertise<std_msgs::Float64MultiArray> ("grids", 10); //
    laser_sub_ = nh_.subscribe ("hokuyo_scan", 10, &ObstacleAvoidance::laserCallback, this);
    dsr_sub_ = nh_.subscribe ("desired_vel", 10, &ObstacleAvoidance::dsrCallback, this);
    sonar_sub_ = nh_.subscribe ("sonar", 10, &ObstacleAvoidance::sonarCallback, this);
    odom_sub_ = nh_.subscribe ("pose", 10, &ObstacleAvoidance::odomCallback, this);
	pos_sub_ = nh_.subscribe ("user_pos", 10, &ObstacleAvoidance::posCallback, this);
}

void ObstacleAvoidance::laserCallback (const sensor_msgs::LaserScan::ConstPtr& laser_msg) {
    this->scan_msg_ = *laser_msg;
	callbackTime_ = ros::Time::now();	// o tempo da chamada é armazenado
}

void ObstacleAvoidance::dsrCallback (const geometry_msgs::Twist::ConstPtr& desired_vel) {
    this->desired_vel_ = *desired_vel;
}

void ObstacleAvoidance::sonarCallback (const p2os_msgs::SonarArray::ConstPtr& sonar_msg) {
    this->sonar_msg_ = *sonar_msg;
}

void ObstacleAvoidance::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg) {
	this->odom_msg_ = *odom_msg;
}

void ObstacleAvoidance::posCallback (const geometry_msgs::Point::ConstPtr& pos_msg) {
	
	if (finishedMapping_ && !finishedMoving_ && takeCommand_) {
	
		pos_msg_ = *pos_msg;	// cria uma configuração final do robô com os dados do usuário
		Point targetPoint (pos_msg_.x, pos_msg_.y);
		targetPoint.yaw = pos_msg_.z;
		
		std::cout << "<x, y, θ> = <" << targetPoint.x << ", " << targetPoint.y << ", " << targetPoint.yaw << ">\n";
		
		int targetNode = retrieveNode (Graph_, targetPoint);
		
		// abaixo eu faço uma breve verificação dos dados
		if (targetNode == 0) {	// se nenhum nó for encontrado...
			std::cout << "Este ponto não pertence aos corredores do CIC mapeados! Entre novamente com os três dados.\n";
			std::cout << "[AGUARDANDO MENSAGEM]\n";
		} else if (std::abs(targetPoint.yaw) > 180.0) {	// se a orientação for inválida...
			std::cout << "A orientação deve estar entre -180.0 e 180.0! Entre novamente com os três dados.\n";
			std::cout << "[AGUARDANDO MENSAGEM]\n";
		} else {
			std::cout << "[MENSAGEM ACEITA]\n";
			receivedValidMsg_ = 1;
			target_ = targetPoint;
			targetNode_ = targetNode;
		}
		
	} else
		std::cout << "[A FUNÇÃO NÃO ESTÁ ACEITANDO MENSAGENS]\n";
	
}

/*	publishMatrix: publica a matriz da grade no tópico /grids.	*/
void publishMatrix (const Grid& grid, int currentNode, const ros::Publisher& grid_pub_, int ri, int rj) {
	
	int width = grid.matrix.size(),
		height = grid.matrix[0].size();
	
	// aqui eu coloco algumas outras informações dentro de uma string que vai ser publicada junto a essa matriz. veja que isso não é a solução mais elegante para publicar a grade de ocupação, visto que eu poderia ter usado nav_msgs/OccupancyGrid como tipo de mensagem em /grids. de qualquer forma, essas informações são usadas pra nomear e criar as imagens das grades
	std::stringstream sswidth;
	sswidth << currentNode << " " << grid.iterNum << " " << ri << " " << rj << "\n";
	
	// o tipo do tópico /grids é std_msgs/Float64MultiArray
	std_msgs::Float64MultiArray gridMatrix;
	
	// a matriz tem duas dimensões, então duas dimensões são adicionadas ao campo dim
	gridMatrix.layout.dim.push_back (std_msgs::MultiArrayDimension());
	gridMatrix.layout.dim.push_back (std_msgs::MultiArrayDimension());
	
	// a primeira dimensão é a da largura, a mais externa
	gridMatrix.layout.dim[0].label = sswidth.str(); // idealmente, eu colocaria simplesmente a string "width" aqui,
	gridMatrix.layout.dim[0].size = width;	// mas como eu precisava passar mais algumas informações e não tinha campos
	gridMatrix.layout.dim[0].stride = width * height;	// prontos pra isso, resolvi colocar as informações na string
	
	// a segunda dimensão é a altura
	gridMatrix.layout.dim[1].label = "height";
	gridMatrix.layout.dim[1].size = height;
	gridMatrix.layout.dim[1].stride = height;
	
	// abaixo a matriz da grade é traduzida num vetor
	gridMatrix.layout.data_offset = 0;
	std::vector<double> tempMatrix (width * height, 0);
	for (int j = 0; j < height; ++j) {
		for (int i = 0; i < width; ++i) {
			tempMatrix[i+j*width] = grid.matrix[i][j];
		}
	}
	
	gridMatrix.data = tempMatrix;
	grid_pub_.publish (gridMatrix);
	
}











/*		ALGORITMO		*/

void ObstacleAvoidance::algorithm() {
	
	Velocity robotVel;
    robotVel.lin = desired_vel_.linear.x;
    robotVel.ang = desired_vel_.angular.z;
	
	Point robotPos (odom_msg_.pose.pose.position.x, odom_msg_.pose.pose.position.y);
	robotPos = robotPos + posOffset;
	robotPos.yaw = tf::getYaw(odom_msg_.pose.pose.orientation) * 180.0/M_PI;
	
	std::vector<float> laser = scan_msg_.ranges;
	
	// aqui o programa calcula a diferença entre o tempo atual e o tempo em que callback foi chamada
	ros::Time rightNow = ros::Time::now();
	double lag = (rightNow - callbackTime_).toSec();
	
	int laserIsWorking = 1;
	
	// se o delay for maior do que 0.5, a mensagem é muito antiga e nesse caso, o laser provavelmente não está funcionando.
	if (lag > 0.5) {	
		laserIsWorking = 0;
		robotVel.lin = 0;
		robotVel.ang = 0;
	}
	
	// estava usando esta verificação antes de checar o tempo da mensagem, mas ela não é muito versátil
	if (laser.empty()) {
		robotVel.lin = 0;
		robotVel.ang = 0;
	}
	
	// aqui o programa percorre e mapeia os nós
	if (!laser.empty() && !finishedMapping_ && laserIsWorking) {
		
		// 
		// DETERMINAÇÃO DA VELOCIDADE DO ROBÔ
		// 
		
		// um ponto em coordenadas polares que representa o ponto mais próximo do robô é obtido
		std::vector<float> closestObs = findClosestObstacle (laser);
		
		// uma velocidade de evasão é determinada
		Velocity evadeVel = evadeObject (laser, robotPos, Path_[step_], closestObs);
		
		// a velocidade associada ao comportamento de seguir um alvo é determinada
		Velocity followVel = followGoal (robotPos, Path_[step_]);
		
		// soma os dois comportamentos, de seguir o alvo e desviar de objetos
		robotVel = manageVel (evadeVel, followVel, closestObs);
		
		// 
		// VERIFICAÇÃO DO CUMPRIMENTO DOS OBJETIVOS
		// 
		
		// aqui a distância até o alvo é encontrada e se o robô atingiu o objetivo, ele vai para o próximo alvo
		if (pdist (robotPos, Path_[step_]) < 0.4) {
			step_++;
			spin_ = 1;	// spin_ = 1 indica que o robô tem que olhar ao seu redor
		}
		
		if (spin_) {
			Velocity turningVel = lookAround (robotPos, Path_[step_], spin_);	// spin_ é alterado por referência
			robotVel = manageVel (evadeVel, turningVel, closestObs);
		}
		
		// 
		// CONSTRUÇÃO DA GRADE DE OCUPAÇÃO
		// 
		
		int prevNode = currentNode_;	// prevNode armazena currentNode_ da iteração anterior
		currentNode_ = retrieveNode (Graph_, robotPos);
		std::cout << "Current node: " << currentNode_ << "\n";
		
		// a cada mudança de nó, o robô salva a matriz do nó passado
		if (prevNode != currentNode_) {
			printGridToFile (Map_[prevNode-1], prevNode);
			writeMatrixOnImage (Map_[prevNode-1].matrix, prevNode, 20);
		}
				
		int robotCelli,
			robotCellj;	// são as posições do robô da matriz
		
		constructOccupancyGrid (Map_[currentNode_-1], laser, robotPos, robotCelli, robotCellj);
		
 		printOccupancyGrid (Map_[currentNode_-1].matrix, robotCelli, robotCellj, currentNode_);
		
		publishMatrix (Map_[currentNode_-1], currentNode_, grid_pub_, robotCelli, robotCellj);
		
		if ((step_) == Path_.size()) {
			finishedMapping_ = 1;
			std::cout << "[PROCESSO DE MAPEAMENTO TERMINADO]\n";
		}
		
		// se o mapeamento estiver pronto, salva grade atual e configura o programa para receber comandos de posição
		if (finishedMapping_) {
			printGridToFile (Map_[currentNode_-1], currentNode_);
			writeMatrixOnImage (Map_[currentNode_-1].matrix, currentNode_, 20);
			takeCommand_ = 1;
			step_ = 0;
			robotVel.lin = 0;
			robotVel.ang = 0;
			printRequestMsg (currentNode_);
		}
	
	// aqui o programa já terminou o mapeamento e o robô vai receber um comando de posição
	} else if (!laser.empty() && finishedMapping_ && laserIsWorking) {
		
		if (takeCommand_) {
			
			// se o robô receber um comando válido, o programa limpa o caminho gerado anteriormente e gera um novo caminho
			if (receivedValidMsg_) {
				Path_.clear();
				Sequence_ = searchGraph (Graph_, currentNode_-1, targetNode_-1);
				constructPathFromPosition (Path_, Sequence_, target_);
				takeCommand_ = 0;
			}
		
		// depois de ler um comando de posição, o robô segue o caminho criado. o molde desse trecho é o mesmo do anterior
		} else if (!finishedMoving_) {	
			
			std::vector<float> closestObs = findClosestObstacle (laser);
			Velocity evadeVel = evadeObject (laser, robotPos, Path_[step_], closestObs);
			Velocity followVel = followGoal (robotPos, Path_[step_]);
			robotVel = manageVel (evadeVel, followVel, closestObs);
			
			if (pdist (robotPos, Path_[step_]) < 0.4)
				step_++;
			
			if ((step_) == Path_.size()) {
				finishedMoving_ = 1;
				spin_ = 1;	// aqui, spin_ = 1 indica que o robô precisa ajustar sua orientação final
				std::cout << "[PROCESSO DE MOVIMENTAÇÃO FINALIZADO]\n";
			}
			
		} else if (spin_) {
			
			// pra corrigir a posição eu também coloquei o desvio de obstáculos. talvez não precisasse, mas achei mais seguro
			std::vector<float> closestObs = findClosestObstacle (laser);
			Velocity evadeVel = evadeObject (laser, robotPos, Path_[step_], closestObs);
			Velocity turningVel = finalAdjustment (robotPos, target_, spin_);	// lembrando que spin_ é alterado por referência
			robotVel = manageVel (evadeVel, turningVel, closestObs);
			
			if (!spin_)
				std::cout << "[AJUSTE FINALIZADO]\n";
		}	
		
	}
	
    command_vel_.linear.x = robotVel.lin;
    command_vel_.angular.z = robotVel.ang;
	
}

void ObstacleAvoidance::spin() {
	
	ros::Rate loop_rate(50);
	
	loop_rate.sleep();
	ros::spinOnce();

	Point initialPos (odom_msg_.pose.pose.position.x, odom_msg_.pose.pose.position.y);
	initialPos = initialPos + posOffset;	// posOffset permite o robô começar de qualquer lugar do mapa

	// abaixo as representações são construídas
	constructMap (Map_);
	constructGraph (Graph_);
	currentNode_ = retrieveNode (Graph_, initialPos);
	constructPathFromFile (Path_, Graph_, currentNode_);
	
	// aqui o programa é configurado para entender que nada foi concluído
	step_ = 0;
	finishedMapping_ = 0;
	finishedMoving_ = 0;
	spin_ = 0;
	takeCommand_ = 0;
	receivedValidMsg_ = 0;
	
    while (ros::ok()) {
        ros::spinOnce();
        algorithm();
        vel_pub_.publish (command_vel_);
        loop_rate.sleep();
    }
	
}

ObstacleAvoidance::~ObstacleAvoidance() {}
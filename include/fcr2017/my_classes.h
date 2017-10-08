/*      DECLARAÇÕES DOS TIPOS        */

// facilitando minha vida
typedef std::vector< std::vector<double> > matrix_t;

// tipos auxiliares
struct Velocity;
struct Point;

// tipos criados para implementar a renderização do bitmap
struct BitmapFileHeader;
struct BitmapInfoHeader;

// tipos criados para implementar a grade de ocupação
struct Grid;

// tipos criados para implementar o mapa topológico
struct Edge;
struct Node;











/*      DEFINIÇÕES DOS TIPOS        */ 

// esta estrutura guarda uma informação sobre a velocidade do robô. mais precisamente, a velocidade linear e a angular.
struct Velocity {
	/*	membros	*/
	double lin, ang;	// estas variáveis armazenam comandos de velocidade dados por alguma função
};

// não tem muito o que explicar. ela me ajuda em quase tudo, especialmente na montagem da grade de ocupação
struct Point {
	/*	membros	*/
	double x, y;	// além de agir como um ponto, Point pode ser interpretado como um vetor
	double yaw;	// opcionalmente, o ângulo da configuração pode ser armazenado em Point
	/* 	funções	*/
	Point (double a, double b): x(a), y(b) {};
	Point () {};
};

// aqui eu tenho as duas structs usadas pra ajudar na implementação da renderização do bitmap a partir da grade.
// obs: não sei se renderizar é o termo técnico correto (pelo que pesquisei, é), mas se não for... ¯\_(ツ)_/¯
// esta é a estrutura do fileheader de um arquivo bitmap, e os nomes tentam ser autoexplicativos.
// peço pra que dê uma olhada no artigo da Wikipedia sobre o formato bitmap se não souber o que é cada coisa (se quiser saber, né)
// lá você vai encontrar uma descrição detalhada do formato e vai ajudar a entender o que eu fiz.
struct BitmapFileHeader {
    /*  membros */
    unsigned char signature [2],
        fileSize [4],
        reserved1 [2],
        reserved2 [2],
        offsetToPixelArray [4];
};

// esta é a estrutura do infoheader do arquivo bitmap.
// existem no total 7 infoheaders que eu poderia ter usado, mas escolhi usar esse simples,
// de só 40 bytes, pra facilitar minha vida rsrs. os nomes tentam ser autoexplicativos.
struct BitmapInfoHeader {
	/*  membros */
    unsigned char headerSize [4],
        bitmapWidth [4],
        bitmapHeight [4],
        numberOfColorPlanes [2],
        bitsPerPixel [2],
        compressionMethod [4],
        imageSize [4],
        horizontalPixelsPerMeter [4],
        verticalPixelsPerMeter [4],
        numberOfColorsInCollorPallete [4],
        numberOfImportantColors [4];
};

// estrutura que armazena a matrix da grade de ocupação e algumas outras características importantes
struct Grid {
	/*	membros	*/
	matrix_t matrix;	// esta é a matrix que guardará a grade
	int width, height,	// informações sobre a grade
		iterNum;	// quantas iterações já se passaram desde que o processamento de uma grade começou
	Point center;	// centro da grade
	/*	funções	*/
	Grid (double a, double b, double w, double h, int nodeNum): width(w), height(h) {
		
		Point tempPoint (a, b);
		center = tempPoint;
		
		std::stringstream fileName;
		fileName << "grids/node" << nodeNum << ".txt";
		std::ifstream nodeFile (fileName.str().c_str(), std::ios::in);
		
		matrix.resize (width, std::vector<double> (height, 0.5));
		
		if (nodeFile.is_open()) {
			int width, height;
			nodeFile >> width >> height;
			for (int j = height-1; j > -1; --j)
				for (int i = 0; i < width; ++i)
					nodeFile >> matrix[i][j];
		}
		
		iterNum = 2;
	};
};

// é um arco de um grafo, que liga um nó a outro
struct Edge {
	/*	membros	*/
	Node* pointTo;	// ponteiro para algum nó
	double weight;	// peso do arco
	/*	funções	*/
	Edge (Node* pNode, double w): pointTo(pNode), weight(w) {};
};

// é um nó do grafo
struct Node {
	/*	membros	*/
	int index;	// permite obter o índice de um nó no vetor Graph. é útil na função searchGraph
	double x, y,	// coordenadas centrais do grafo. os valores são calculados a partir de uma média dos limites da região
		xbound1, xbound2, ybound1, ybound2;	// limites da região que define o grafo
	std::vector<Edge> edges;	// vetor de arcos do nó
	/*	funções	*/
	Node () {};
	Node (double x1, double x2, double y1, double y2, int i): xbound1(x1), xbound2(x2), ybound1(y1), ybound2(y2), index(i) {
		x = (x1 + x2)/2;
		y = (y1 + y2)/2;
	}
	int isInside (Point pos) const {	// verifica se o robô está dentro do nó, comparando a posição do robô com os limites da região
		return ((pos.x > xbound1) && (pos.x < xbound2) && (pos.y > ybound1) && (pos.y < ybound2));
	}
};
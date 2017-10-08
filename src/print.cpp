#include "fcr2017/obstacle_avoidance.h"

matrix_t globalMatrix;  // matrix lida de /grids

int nodeNum,    // nó atual
    iterNum,    // iteração atual
    ri, rj;     // posição do robô na matriz

void printCallback (const std_msgs::Float64MultiArray::ConstPtr& msg) {
    
    int width = (*msg).layout.dim[0].size,
        height = (*msg).layout.dim[1].size;
    
    // aqui as informações adicionais são lidas da string label de dim[0]
    std::stringstream ssnode ((*msg).layout.dim[0].label);
    ssnode >> nodeNum >> iterNum >> ri >> rj;
    
    // o vetor de dados é traduzido para uma matriz
    std::vector<double> vectorData = (*msg).data;
    matrix_t matrix (width, std::vector<double> (height, 0));
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            matrix[i][j] = vectorData[i+j*width];
        }
    }
    
    globalMatrix = matrix;
    
}

/*  printGridToFile: imprime as matrizes recebidas num arquivo texto. É uma outra versão da função principal, já que a numeração dos arquivos aqui é diferente. */
void printGridToFile (const matrix_t& matrix) {
	
	std::stringstream fileName; // a numeração dos nomes é diferente aqui
	fileName << "grids/node" << nodeNum << "-" << iterNum << ".txt";
	std::ofstream textFile (fileName.str().c_str(), std::ios::out | std::ios::trunc);
	
	std::streamsize ss = textFile.precision();
	textFile << std::setprecision (3);
	textFile << std::fixed;
	
	textFile << matrix.size() << " " << matrix[0].size() << "\n";
	
	for (int j = matrix[0].size()-1; j > -1; --j) {
		for (int i = 0; i < matrix.size(); ++i)
			textFile << matrix[i][j] << " ";
		textFile << "\n";
	}
	
	textFile.unsetf (std::ios::floatfield);
	textFile << std::setprecision(ss);
	
	textFile.close();
	
}

/*  writeMatrixOnImage: escreve as matrizes recebidas num bitmap. Foi refeita aqui pelo mesmo motivo da função acima.   */
void writeMatrixOnImage (const matrix_t& matrix, int scale) {
    
	std::stringstream imageName;    // o nome das imagens também é diferente
	imageName << "bitmaps/image" << nodeNum << "-" << iterNum << ".bmp";
	std::ofstream image (imageName.str().c_str(), std::ios::binary | std::ios::out | std::ios::trunc);
	
    int width = matrix.size()*scale,
        height = matrix[0].size()*scale,
        pixArrSize = (24*width+31)/32 * 4 * height,
        fileSize = pixArrSize + 0x36;
    
    BitmapFileHeader fileHeader = {
        {0x42, 0x4D},
        {fileSize&0xFF, (fileSize&0xFF00)>>8, (fileSize&0xFF0000)>>16, (fileSize&0xFF000000)>>24},
        {0x00, 0x00},
        {0x00, 0x00},
        {0x36, 0x00, 0x00, 0x00}
    };
    
    BitmapInfoHeader infoHeader = {
        {0x28, 0x00, 0x00, 0x00},
        {width&0xFF, (width&0xFF00)>>8, (width&0xFF0000)>>16, (width&0xFF000000)>>24},
        {height&0xFF, (height&0xFF00)>>8, (height&0xFF0000)>>16, (height&0xFF000000)>>24},
        {0x01, 0x00},
        {0x18, 0x00},
        {0x00, 0x00, 0x00, 0x00},
        {pixArrSize&0xFF, (pixArrSize&0xFF00)>>8, (pixArrSize&0xFF0000)>>16, (pixArrSize&0xFF000000)>>24},
        {0x13, 0x0B, 0x00, 0x00},
        {0x13, 0x0B, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x00},
    };
    
    image.write ((char*) &fileHeader, sizeof(BitmapFileHeader));
    image.write ((char*) &infoHeader, sizeof(BitmapInfoHeader));
    
	for (int j = 0, n = 0; j < height; ++j) {
        for (int i = 0, m = 0; i < width; ++i) {
            unsigned char pixel [3],
                shade = 0xFF * (1 - matrix[m][n]);
            if (m == ri && n == rj) {
                pixel[0] = 0x00;
                pixel[1] = 0x00;
                pixel[2] = 0xFF;
            } else {
			    pixel[0] = shade;
                pixel[1] = shade;
                pixel[2] = shade;
            }
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

int main (int argc, char** argv) {
    
	ros::init (argc, argv, "print");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe ("grids", 10, printCallback);
    
    ros::Rate loop_rate (0.5);
    
    while (ros::ok()) {
        if (!globalMatrix.empty()) {
            printGridToFile (globalMatrix);
            writeMatrixOnImage (globalMatrix, 20);
        } else
            std::cout << "[NO DATA STREAM]\n";
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
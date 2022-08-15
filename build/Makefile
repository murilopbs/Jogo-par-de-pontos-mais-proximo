all: compilar linkar

compilar:
	g++ -I -c ../Sources/teste.cpp -c ../Sources/Jogo.cpp -c ../Sources/Jogador.cpp -c ../Sources/menor.cpp -c ../Sources/Menu.cpp -lsfml-graphics -lsfml-window -lsfml-system

linkar:
	g++ teste.o Jogo.o Jogador.o menor.o Menu.o -o teste -l sfml-graphics -l sfml-window -l sfml-system -l sfml-audio && rm Jogo.o && rm teste.o && rm Jogador.o && rm menor.o && rm Menu.o


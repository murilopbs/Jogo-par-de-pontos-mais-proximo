#include "../Headers/Jogo.h"
#include "../Headers/Jogador.h"
#include "../Headers/menor.h"
#include "../Headers/Menu.h"
#include <SFML/Network.hpp>
#include <SFML/Audio.hpp>

int main(){   

    sf::Music musica;
    if(!musica.openFromFile("../Musica/Minecraft.ogg")){
        std::cout << "Erro ao abrir a música\n";
    }
    musica.play();
    Menu menu;
    int opcaoMenu;
    int dificuldade = 0;
    opcaoMenu = menu.fazMenu();
    switch (opcaoMenu){
        case 1:
            break;
        case 2:
            dificuldade = menu.fazOpcoes();
            break;
        case 3:
            return 0;
        default:
            return 0;
    }

    Jogo jogo;
    
    int venceu = 5;

    Jogador jogador;

    jogador.dificuldade = dificuldade;

    jogador.fase = 0;

    Menor menor;

    jogador.iniciaSeguro();

    int contador = 0;
    

    while(jogo.running()){
        jogo.update();

        /// Movimenta a cabeça da cobrinha para a direção atual
        jogador.Movimenta(jogo.direcao);

        /*if(jogador.posicao() == jogador.posicaoComida())
        {
            jogador.pegouComida();

            jogo.fps+=0.5;
            jogo.setFrtL(jogo.fps);
        }*/

        /// Se a cabeça da cobrinha sair dos limites da janela, a cobrinha é redirecionada para o outro lado
        jogador.confereBordas();      

        //menor.calculaMinimo(jogador.posicao(), jogador.posicaoInimigo());

        /// Se a cabeça da cobrinha bater no seu corpo, ela morre
        venceu = jogador.morreu();
        if(venceu == 1){ //jogador realmente venceu
            jogador.fase += 1;
            jogo.proximaFase(jogador.fase);
            jogo.fps += 0.5;
            jogo.setFrtL(jogo.fps);
            jogador.iniciaSeguro();
            //break;
        }else if(venceu == 0){//jogador perdeu tudo
            bool escolhaFim = jogo.perdeuTudo(jogador.fase);
            if(escolhaFim == true){
                jogo.~Jogo();
                opcaoMenu = menu.fazMenu();
                switch (opcaoMenu){
                    case 1:
                        venceu = 5;
                        jogador.fase = 0;
                        jogo.fps = 4;
                        jogo.setFrtL(jogo.fps);
                        jogo.voltaMenu();
                        contador = 0;
                        jogador.iniciaSeguro();
                    case 2:
                        dificuldade = menu.fazOpcoes();
                        jogador.dificuldade = dificuldade;
                        venceu = 5;
                        jogador.fase = 0;
                        jogo.fps = 4;
                        jogo.setFrtL(jogo.fps);
                        jogo.voltaMenu();
                        contador = 0;
                        jogador.iniciaSeguro();
                        break;
                    case 3:
                        return 0;
                    default:
                        return 0;
                }
            }
            else{
                break;
            }
        }
        
        if(contador < 4){
            ++contador;
        }else{
            jogador.movimentaInimigo();
            contador = 0;
        }
        jogo.desenhaComida(jogador.retornaInimigo()); /// Desenha a comida

        /*for(int i=0; i<jogador.retornaTamanho(); i++)
        {
            /// Um efeito legal na cobrinha

            jogo.desenhaCobra(jogador.fazPersonagem(i));
        }*/

        jogo.desenhaJogador(jogador.fazPersonagem());

        jogo.render();
    }
    if(venceu == 1){
        std::cout << "Você venceu\n"; 
    }
    else if(venceu == 0){
        std::cout << "Você perdeu\n";
    }else if(venceu == 5){
        std::cout << "Falhou lkkkkk \n";
    }
    return 0;
}
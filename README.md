# Jogo-par-de-pontos-mais-proximo
Jogo em C++ que utiliza o Problema do par de pontos mais próximo.
# Sobre
O problema do par de pontos mais próximo consiste em uma entrada de n pontos no plano,
determinar dois deles que estão à distância mínima. Uma solução seria um algoritmo quadrático que testa todas as combinações possíveis. Não obstante, como a cada frame do jogo é checado qual o par de pontos mais próximo, essa complexidade acaba não endo a ideal. Utilizando a técnica de dividir e conquistar, conseguiremos uma complexidade O(n*logn).

## Screenshots
<table>
    <tr>
        <td>Tela inicial</td><td>Dificuldade Fácil</td><td>Dificuldade Difícil</td>
    </tr>
    <tr>
        <td><img src="/1.png" width="200"></td><td><img src="/2.png" width="200"></td><td><img src="/3.png" width="200"></td>
    </tr>
</table>

# Instalação 
**Linguagem**: C++<br>
**Framework**: SFML<br>
Sistema que utiliza APT:
##### Abra o terminal
- Baixe o projeto e entre no diretório
> $ git clone https://github.com/projeto-de-algoritmos/DC_Chega_Perto && cd DC_Chega_Perto
- Instale os requisitos
> $ sudo apt-get install libsfml-dev && sudo apt-get install build-essential
- Entre na pasta build
> $ cd build
- Compile o projeto
> $ make
- Rode o projeto
> $ ./Jogo_Binario
- Divirta-se

# Uso
Após rodar o projeto, é importante saber que todo o jogo é jogado usando as setas do teclado. Use as setas no menu, e aperte 'enter' para escolher a opção.

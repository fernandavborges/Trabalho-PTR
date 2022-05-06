Para execução do código de teste, é preciso que todos esses arquivos estejam no diretório do projeto:

1. sim.py
2. simConst.py
3. API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) ou "remoteApi.so" (Linux)
4. cenaTeste.ttt
4. Pasta pyRTOS
5. Test.py 


O arquivo Test.py é onde se encontra todo o remote API com implementação do RTOS.
O arquivo cenaTeste.ttt é uma cena do Coppelia e está configurada para realizar o teste presente em Test.py. Caso o código Test.py seja rodado, espera-se que alguns modelos de "mosquito" sejam apagados da cena teste (simbolizando sua morte), logo, para uma nova execução, é preciso abrir novamente a cenaTeste.ttt baixada com o projeto ou retornar ao estado inicial dos modelos utilizando o ctrl+Z até que todos os mosquitos retornem à cena. 

Para compilação e execução: python3 Test.py
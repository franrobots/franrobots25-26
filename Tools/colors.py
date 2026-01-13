from random import randint
colors = {
    -2: "black",
    -1: "red",
    0: "yellow",
    1: "greem",
    2: "blue"
}

while True:
    number = []
    actualColors = []

    for i in range(5):
        number.append(randint(-2, 2))
        
    for i in number:
        actualColors.append(colors[i])

    print(number)
    print(actualColors)
    print("total:", sum(number))
    input ("Aperte enter para continuar . . .")
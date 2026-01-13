import os
import random
import string

def gerar_nome_aleatorio(tamanho=6):
    """Gera uma string aleatória com letras e números."""
    caracteres = string.ascii_letters + string.digits
    return ''.join(random.choice(caracteres) for _ in range(tamanho))

def renomear_arquivos(pasta):
    """Renomeia todos os arquivos da pasta com nomes aleatórios."""
    if not os.path.isdir(pasta):
        print("❌ Pasta não encontrada!")
        return

    arquivos = os.listdir(pasta)
    if not arquivos:
        print("⚠️ Nenhum arquivo encontrado na pasta.")
        return

    for arquivo in arquivos:
        caminho_antigo = os.path.join(pasta, arquivo)
        if os.path.isfile(caminho_antigo):
            extensao = os.path.splitext(arquivo)[1]  # ex: ".png"
            novo_nome = gerar_nome_aleatorio() + extensao
            caminho_novo = os.path.join(pasta, novo_nome)

            # Evita nomes duplicados
            while os.path.exists(caminho_novo):
                novo_nome = gerar_nome_aleatorio() + extensao
                caminho_novo = os.path.join(pasta, novo_nome)

            os.rename(caminho_antigo, caminho_novo)
            print(f"✅ {arquivo} → {novo_nome}")

    print("\n🎉 Todos os arquivos foram renomeados com sucesso!")

# ----------------------------
# COMO USAR:
# Basta colocar o caminho da pasta entre aspas abaixo, por exemplo:
# renomear_arquivos(r"C:\Users\SeuUsuario\Desktop\H_semBorda")
# ----------------------------

if __name__ == "__main__":
    pasta = input("Digite o caminho da pasta com as fotos: ").strip()
    renomear_arquivos(pasta)

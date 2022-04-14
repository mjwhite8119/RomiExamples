import tarfile
import os


def build():
    """
    Puts all Python files into a .zip for easy uploading to Raspberry Pi.
    :return: None
    """
    files = [f for f in os.listdir('.') if os.path.isfile(f) and f.endswith(".py")]
    files.sort()
    files.append("runCamera")

    with tarfile.open("wpilib.tar.gz", "w:gz") as tar:
        for file in files:
            tar.add(file)
    print("wpilib.tar.gz generated. Ready to upload. Files included:", end='\n\t')
    print(*files, sep="\n\t")


if __name__ == "__main__":
    build()

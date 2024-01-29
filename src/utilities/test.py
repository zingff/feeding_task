import pygame

def play_wav(file_path):
    pygame.mixer.init()
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

if __name__ == "__main__":
    file_path = "/home/zing/mealAssistiveRobot/sla_ws/src/feeding_task/config/voice_response/youCanEat.wav"
    play_wav(file_path)

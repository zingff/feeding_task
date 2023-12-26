import speech_recognition as sr

def speechtotext(language):
    recognizer = sr.Recognizer()

    # Capture audio from a microphone
    with sr.Microphone() as source:
        print("请您说话:")
        audio = recognizer.listen(source)

    # Recognize speech and convert to text
    try:
        text = recognizer.recognize_google(audio, language=language)
        print("语音识别结果: " + text)
        return text
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition; {0}".format(e))
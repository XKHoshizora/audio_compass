#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyttsx3

def list_voices_to_file(output_file="voices.txt"):
    # 初始化 pyttsx3 引擎
    engine = pyttsx3.init()

    # 获取所有可用的语音
    voices = engine.getProperty('voices')

    # 打开文件以写入语音信息
    with open(output_file, "w", encoding="utf-8") as file:
        file.write("Available Voices:\n\n")

        for voice in voices:
            file.write(f"ID: {voice.id}\n")
            file.write(f"Name: {voice.name}\n")
            file.write(f"Languages: {voice.languages}\n")
            file.write(f"Gender: {voice.gender}\n")
            file.write(f"Age: {voice.age}\n")
            file.write("-" * 40 + "\n")

    print(f"Voice information has been written to {output_file}")

if __name__ == "__main__":
    list_voices_to_file()

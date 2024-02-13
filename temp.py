import ollama

with open("Modelfile", "r") as f:
    ollama.create(model = "temp", modelfile= f.read())

print(ollama.generate(model='temp', prompt='What\'s the weather like right now in London?', options={"stop": ["\nThought:"]}))

ollama.delete("temp")
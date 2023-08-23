# # import empath
# from hazm import word_tokenize
# # from translate import Translator

# # # Create an Empath analyzer
# # ea = empath.Empath()
# # translate = Translator(to_lang="en" ,from_lang="fa")
# # # Create a sentence to analyze
# # text = "من خیلی خوشحالم"
# # text = translate.translate(text)
# # print(text)
# # # Tokenize the text using hazm
# # tokens = word_tokenize(text)
# # print(tokens)
# # # Analyze the emotions in the text using Empath
# # emotions = ea.analyze (' '.join(tokens), normalize=True)

# # # Print the emotions
# # print([i for i in emotions if emotions[i] > 0])


# from transformers import pipeline, AutoTokenizer, TFAutoModelForSequenceClassification, TFTrainer,TFTrainingArguments
# import datasets
# import pandas as pd
# from matplotlib import pyplot as plt
# from sklearn.model_selection import train_test_split
# from sklearn.preprocessing import StandardScaler
# from sklearn.ensemble import RandomForestClassifier
# from sklearn.feature_extraction.text import CountVectorizer

# class Classification():
#     def __init__(self, file_name="NRC-emotion-lexicon-wordlevel-persian-v0.92.xlsx", result ='value'):
#         df = pd.read_excel(file_name)
#         # Get the data ready for furthur processing
#         self.len_cols = len(df.columns)
#         std_scaler = StandardScaler()
#         temp_data = df.iloc[:,0]

#         df_values = temp_data.values
#         print(df_values)
#         # std_df = std_scaler.fit_transform(temp_data)
#         # df_normalize = pd.DataFrame(std_df)
#         # df_normalize
#         self.vectorizer = CountVectorizer(max_features=1500, min_df=5, max_df=0.7)
#         X = self.vectorizer.fit_transform(df_values).toarray()
#         self.x = X
#         self.y = df[result]
#         self.iterrows = df.itertuples()
#         self.df = df
#         self.col = df.columns

    
#         for i in range(self.len_cols-1):
#             for row in self.col:
#                 try:
#                     plt.scatter(x[i],y)
#                     plt.xlabel(row)
#                     plt.ylabel(result)
#                 except:
#                     break
#     def classify(self, **kwargs):
#         self.df.reset_index()
#         print([self.x,self.y])
#         self.alg = RandomForestClassifier(n_estimators=1000, random_state=0)
#         self.alg.fit(self.x,self.y)
#         return [self.x,self.y]

#     def prediction(self, item=[[]]):
#         self.df.reset_index()
#         pred = self.alg.predict(item)
#         return pred


# clas = Classification()
# clas.classify()
# # Load the pre-trained tokenizer for Persian (Farsi)
# # tokenizer = AutoTokenizer.from_pretrained('HooshvareLab/bert-fa-base-uncased')

# # Use the fine-tuned model to perform sentiment analysis
# text = "این فیلم بسیار جذاب و دیدنی است"
# encoding = word_tokenize(text)
# # output = model(**encoding)
# # result = output.logits.argmax().item()

# # Print the sentiment analysis result
# print(clas.prediction(encoding))
# # # Load the pre-trained GPT-Neo tokenizer
# # tokenizer = AutoTokenizer.from_pretrained('EleutherAI/gpt-neo-1.3B')


# # # Load the IMDB movie reviews dataset
# # dataset = datasets.load_dataset('imdb')

# # # Prepare the dataset for training
# # train_dataset = dataset['train'].map(lambda example: {'text': example['text'], 'label': example['label']})

# # # Load the pre-trained GPT-Neo model for sequence classification
# # model = AutoModelForSequenceClassification.from_pretrained('EleutherAI/gpt-neo-1.3B', num_labels=2)

# # # Fine-tune the model on the IMDB dataset
# # trainer = pipeline('text-classification', model=model, tokenizer=tokenizer)
# # trainer.train(train_dataset, max_steps=1000)

# # # Use the fine-tuned model to perform sentiment analysis
# # text = "This movie was really great!"
# # result = trainer(text)

# # # Print the sentiment analysis result
# # print(result)


# import tensorflow as tf
# from transformers import AutoTokenizer, TFAutoModelForSequenceClassification
# from hazm import word_tokenize

# # Load the pre-trained tokenizer and model
# tokenizer = AutoTokenizer.from_pretrained("persiannlp/bert-base-parsbert-emotion")
# model = TFAutoModelForSequenceClassification.from_pretrained("persiannlp/bert-base-parsbert-emotion")

# # Define a function to predict the emotion of a given text
# def predict_emotion(text):
#     # Tokenize the input text
#     tokens = tokenizer.encode_plus(text, max_length=512, padding=True, truncation=True, return_tensors='tf')
    
#     # Perform the model inference
#     output = model(tokens)
#     scores = output.logits.numpy()[0]
    
#     # Convert the scores to probabilities
#     probabilities = tf.nn.softmax(scores).numpy()
    
#     # Return the predicted emotion label and probability
#     emotion_labels = ['Anger', 'Disgust', 'Fear', 'Joy', 'Sadness', 'Surprise', 'Neutral']
#     emotion = emotion_labels[int(tf.argmax(scores))]
#     probability = probabilities[int(tf.argmax(scores))]
    
#     return emotion, probability

import requests


class FarsGPT2():

	def __init__(self,api_model):
		if api_model == "gpt2_fa":
			self.gpt2_fa()

		if api_model == "gpt2_fa_poetry":
			self.gpt2_fa_poetry()
		

	def gpt2_fa(self):
		self.API_URL = "https://api-inference.huggingface.co/models/HooshvareLab/gpt2-fa" 
		self.headers = {"Authorization": "Bearer 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"}

	def gpt2_fa_poetry(self):
	
		self.API_URL = "https://api-inference.huggingface.co/models/HooshvareLab/gpt2-fa-poetry"
		self.headers = {"Authorization": "Bearer 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"}

	def query(self,payload):
			response = requests.post(self.API_URL, headers=self.headers, json=payload)
			return response.json()
			

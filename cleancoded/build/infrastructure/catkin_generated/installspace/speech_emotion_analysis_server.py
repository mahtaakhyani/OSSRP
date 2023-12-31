'''
This module is the server for the speech emotion analysis service. 
It receives the audio data from the client and sends it to the feature extraction service. 
Then, it receives the extracted features in a custom message called AudFeature [which is located in "$(rospack find infrastructure)/msg"] and sends them as a list of floats to the emotion classification module to predict the emotion of the ongoing speech. (emotion_classification.py module is a python tool, not a ROS node. It is located in "$(rospack find infrastructure)/tools"**).

Then, it receives the emotion of the ongoing speech as a string from the emotion classification module and sends it back to the client via the EmoProb service [which is located in "$(rospack find infrastructure)/srv"].

**[It's a machine learning model that can be trained to predict the emotion of the ongoing speech based on the extracted features of the speech. The dataset is located in "$(rospack find infrastructure)/tools/datasets")]
'''

#!/usr/bin/env python
import rospy
from audio_common_msgs.msg import AudioDataStamped as AudioData
from infrastructure.srv import AudFeature, EmoProb



class SpeechEmotionAnalysis(EmotionClassification):
    # Initialize the node and create a service
    rospy.init_node('speech_emotion_analysis_server')

    def __init__(self):
        rospy.Service('speech_emotion_analysis', EmoProb, self.callback)
        # Create a service named speech_emotion_analysis with the EmoProb service type [which is located in "$(rospack find infrastructure)/srv"]
        # The EmoProb service type has two fields:
        # 1. request: audio data (AudioDataStamped message type)
        # 2. response: emotion of the ongoing speech (string)
        EmotionClassification.classify(self,)

    def callback(self, data):
        # Wait for the audio_features service to be available
        rospy.wait_for_service('audio_features')
        try:
            # Create a proxy for the audio_features service to call it
            audfeaturesrv = rospy.ServiceProxy('audio_features', AudFeature)
            # Call the audio_features service and get the extracted features of the speech
            features = [audfeaturesrv(data)]

        # ---------
        # Alternative: Sentiment analysis using Empath (text-based, faster, but less accurate):
            # import empath
            # from hazm import word_tokenize
            # from std_msgs.msg import String

            # # Get the text from the audio data
            # rospy.wait_for_service('speech_to_text')
            # try:
            #     sttsrv = rospy.ServiceProxy('speech_to_text', String)
            #     text = sttsrv(data)
            # except rospy.ServiceException as e:
            #     rospy.logerror("Service call failed: %s"%e)

            # # Create an Empath analyzer
            # ea = empath.Empath()
            # # Tokenize the text using hazm
            # tokens = word_tokenize(text)
            # # Analyze the emotions in the text using Empath
            # emotions = ea.analyze (' '.join(tokens), normalize=True)
        # ---------
            return emotion

        except rospy.ServiceException as E:
            rospy.logerror("Service call failed: %s" % E)

# This is a work of HooshvareLab/bert-fa-base-uncased
# https://huggingface.co/HooshvareLab/bert-fa-base-uncased
# The credit goes to the original authors of the model.

# The NRC-emotion-lexicon classification is text(=transcript) classification not audio feature classification
    # while classifying it with ESD_extracteddata, is audio feature classification.
import pandas as pd
from hazm import word_tokenize
from matplotlib import pyplot as plt
from sklearn.ensemble import RandomForestClassifier
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.preprocessing import StandardScaler
from transformers import (AutoTokenizer, TFAutoModelForSequenceClassification,
                          TFTrainer, TFTrainingArguments, pipeline)


class EmotionClassification():
    def __init__(self, file_name="NRC-emotion-lexicon-wordlevel-persian-v0.92.xlsx", result ='value'):
        df = pd.read_excel(file_name)
        # Get the data ready for furthur processing
        self.len_cols = len(df.columns)
        std_scaler = StandardScaler()
        temp_data = df.iloc[:,0]

        df_values = temp_data.values
        print(df_values)
        # std_df = std_scaler.fit_transform(temp_data)
        # df_normalize = pd.DataFrame(std_df)
        # df_normalize
        self.vectorizer = CountVectorizer(max_features=1500, min_df=5, max_df=0.7)
        X = self.vectorizer.fit_transform(df_values).toarray()
        self.x = X
        self.y = df[result]
        self.iterrows = df.itertuples()
        self.df = df
        self.col = df.columns

    
        for i in range(self.len_cols-1):
            for row in self.col:
                try:
                    plt.scatter(self.x[i],self.y)
                    plt.xlabel(row)
                    plt.ylabel(result)
                except:
                    break
        
    def classify(self, **kwargs):
        self.df.reset_index()
        print([self.x,self.y])
        self.alg = RandomForestClassifier(n_estimators=1000, random_state=0)
        self.alg.fit(self.x,self.y)
        return [self.x,self.y]

    def prediction(self, item=[[]]):
        self.df.reset_index()
        pred = self.alg.predict(item)
        return pred


# OR Load the pre-trained tokenizer for Persian (Farsi)
# tokenizer = AutoTokenizer.from_pretrained('HooshvareLab/bert-fa-base-uncased')

# Use the fine-tuned model to perform sentiment analysis
# text = "این فیلم بسیار جذاب و دیدنی است"
# encoding = word_tokenize(text)
# output = model(**encoding)
# result = output.logits.argmax().item()

# Print the sentiment analysis result
# print(clas.prediction(encoding))


# ---------------------------------------------------------
# *** The following code is the previous version of the classification module. It is not used in the current version.
# ---------------------------------------------------------



# '''
# This module trains the model based on the table :
#     emotion's name | audio features [e.g. pitch, hnr, jitter, shimmer, etc.]
# to predict the emotion of the ongoing speech. [The dataset is located in "$(rospack find infrastructure)/tools/datasets"]
# *** It also includes functions to optimize, plot, and change the model based on the accuracy of the output.

# The output of this module is the predicted emotion of the ongoing speech as a string, in the prediction function.
# '''
# import math

# import numpy as np
# import pandas as pd
# from matplotlib import pyplot as plt
# from sklearn.model_selection import train_test_split
# from sklearn.preprocessing import StandardScaler



# class Classification():
#     def init_plot(self, file_name="datasets.xlsx", sheet= "speech_emotion_analyzer", result ='Index'):
#         df = pd.read_excel(file_name)
#         # Get the data ready for furthur processing
#         self.len_cols = len(df.columns[1:])
#         std_scaler = StandardScaler()
#         temp_data = df.iloc[:,2:self.len_cols+1]

#         df_values = temp_data.values
#         std_df = std_scaler.fit_transform(temp_data)
#         df_normalize = pd.DataFrame(std_df)
#         df_normalize

#         self.x = df_normalize
#         self.y = df[result]
#         self.iterrows = df.itertuples()
#         self.df = df
#         self.col = df.columns

    
#         for i in range(self.len_cols-1):
#             for row in self.col[1:]:
#                 try:
#                     plt.scatter(x[i],y)
#                     plt.xlabel(row)
#                     plt.ylabel(result)
#                 except:
#                     break
#         plt.plot(self.x,self.y)
#         plt.show()


#     def classify(self, algorithm='knn', **kwargs):
#         self.df.reset_index() # Reset the index of the dataframe
#         if algorithm.lower() in ["knn", "kneighborsregressor", "kneighborclassifier"]:
#             from sklearn.neighbors import KNeighborsRegressor
#             from sklearn.neighbors import KNeighborsClassifier
#             self.alg = KNeighborsClassifier()

#         if algorithm.lower() in ["aff","affinity_propagation", "affinitypropagation", "affinity propagation"]:
#             from sklearn.cluster import AffinityPropagation
#             self.alg = AffinityPropagation()

#         if algorithm.lower() in ["agglomerativeclustering", "agglomerative", "agglomerative clustering", "aggclustering"]:
#             from sklearn.cluster import AgglomerativeClustering
#             self.alg = AgglomerativeClustering()

#         if algorithm.lower() in ["birch"]:
#             from sklearn.cluster import Birch
#             self.alg = Birch()

#         if algorithm.lower() in ["dbscan"]:
#             from sklearn.cluster import DBSCAN
#             self.alg = DBSCAN()

#         if algorithm.lower() in ["kmeans", "k-means"]:
#             from sklearn.cluster import KMeans
#             self.alg = KMeans()

#         for param in kwargs.keys():
#             try:
#                 self.alg(kwargs[param])
#             except:
#                 pass

#         self.alg.fit(self.x,self.y) # Train the model
#         return [self.x,self.y]
    

#     def prediction(self, item=[[]]): 
#         # item is the array including the list of an audio clip's extracted features. 
#         #   (e.g. [[165.841,289.356,205.586 ,33.699 , 79.329, 69.195,0.019 , 0.093,16.078 ,1.311]])
#         # The output of this function is the predicted emotion of the audio clip as a string.
#         self.df.reset_index()
#         try:
#             test = [[i[3:] for i in self.df.itertuples()][-1]]
#         except:
#             test = item
#         print(test)
#         pred_emotion_index = self.alg.predict(test)
#         from sklearn.neighbors import NearestNeighbors
#         import scipy.sparse as sparse
#         neigh = NearestNeighbors(n_neighbors=2)
#         neigh.fit(self.x,self.y)
#         NearestNeighbors(n_neighbors=2)
#         A = neigh.kneighbors_graph(self.x)
#         # plt.spy(A)

#         print(self.alg.predict_proba(test))
#         pred_emotion_name = self.df.reset_index()['Emotion'][int(round(pred_emotion_index[0]))]
        
#         return pred_emotion_name




# class Optimizer(Classification):
#     def __init__(self):
#         self.index = []
#         self.error = []
#         temp = Classification()
#         self.x = temp.x
#         self.y = temp.y
#         self.len_cols = temp.len_cols

#     def knn_opt(self,*dataframes):
#         from sklearn.neighbors import KNeighborsRegressor
#         (x , y) = (self.x,self.y)
#         self.knn = KNeighborsRegressor()
#         self.raiseError(x,y)
#         n = len(y)
#         for i in range(1, self.len_cols-2):
#             self.knn.n_neighbors = i
#             self.knn.fit(x, y)

#             y_pre = self.knn.predict(x)
#             rss = sum((y_pre - y)**2)
#             rse = math.sqrt(rss/(n-2))

#             self.index.append(i)
#             self.error.append(rse)

#         dif = 0
#         max_dif = 0
#         k = 0
#         for j in range(len(self.error)):
#             k = self.index[j]
#             for m in range(1,len(self.error)-1):
#                 dif = math.fabs(self.error[j] - self.error[j-m])
#                 if dif <= max_dif and dif <=1: 
#                     max_dif = dif
#                     k = self.index[j]
#                 print(dif,max_dif,m,k)
#         self.Errorplot()
#         return (k)

#     def Errorplot(self):
#         plt.plot(self.index, self.error)
#         plt.xlabel('Values of K')
#         plt.ylabel('Emotion Error')
#         plt.title('Graph: K VS Error')

#         plt.show()

#     def raiseError(self,*args):
#         for arg in args:
#             if str(type(arg)) not in ["<class 'pandas.core.frame.DataFrame'>", 
#                             "<class 'pandas.core.series.Series'>"]:
#                 raise "Invalid DataFrame type"



# if __name__=="__main__":
#     clsf = Classification()
#     # clsf.classify(algorithm='aff')
#     # pr = clsf.prediction()
#     # clsf.prediction([[165.841,289.356,205.586 ,33.699 , 79.329, 69.195,0.019 , 0.093,16.078 ,1.311]])
#     # print(pr)
#     clsf.classify()
#     pr = clsf.prediction()
#     print(pr)
#     opt = Optimizer()
#     new_k = opt.knn_opt(clsf)
#     clsf.classify(n=new_k)
#     pr = clsf.prediction()
#     print(pr)

if __name__ == "__main__":
    try:
        stt = SpeechEmotionAnalysis()
        rospy.spin()  # keeps python from exiting until this node is stopped
    except rospy.ROSInterruptException:
        pass

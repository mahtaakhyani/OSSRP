#!/usr/bin/env python
from transformers import GPT2Tokenizer, GPT2LMHeadModel
import torch
from translate import Translator
import rospy
from std_msgs.msg import String

class ConversationService:
    def __init__(self):
        rospy.init_node('conversation_bot_server', anonymous=False)
        rospy.Subscriber('say_sth/bot', String, self.callservice)
        self.pub = rospy.Publisher('say_sth', String, queue_size=10)
        self.bot = ConversationBot()
        rospy.loginfo("Ready to start conversation.")

    def callservice(self, data):
        rospy.loginfo('Got a conversation text on /say_sth/bot. Calling the bot module.')
        try:
            response = self.bot.start_conversation(data)
            self.pub.publish(response)
            
        except rospy.ServiceException as e:
            rospy.logerr("Module call failed: %s"%e)
        

    


class ConversationBot:
    translator_fa= Translator(to_lang="en", from_lang="fa")
    translator_en= Translator(to_lang="fa", from_lang="en")
    tokenizer = GPT2Tokenizer.from_pretrained("af1tang/personaGPT")
    model = GPT2LMHeadModel.from_pretrained("af1tang/personaGPT")
    
    def __init__(self):
        if torch.cuda.is_available():
            self.model = self.model.cuda()
        ## utility functions ##
        self.flatten = lambda l: [item for sublist in l for item in sublist]

        # get personality facts for conversation
        self.personas = []
        for i in range(3):
            response = input(">> Fact %d: "%(i+1))+ self.tokenizer.eos_token
            self.personas.append(response)
        self.personas = self.tokenizer.encode(''.join(['<|p2|>'] + self.personas + ['<|sep|>'] + ['<|start|>']))


    def to_data(self, x):
        if torch.cuda.is_available():
            x = x.cpu()
        return x.data.numpy()

    def to_var(self, x):
        if not torch.is_tensor(x):
            x = torch.Tensor(x)
        if torch.cuda.is_available():
            x = x.cuda()
        return x

    def display_dialog_history(self, dialog_hx):
        for j, line in enumerate(dialog_hx):
            msg = self.tokenizer.decode(line)
            if j %2 == 0:
                print(">> User: "+ msg)
            else:
                print("Bot: "+msg)
                print()

    def generate_next(self, bot_input_ids, do_sample=True, top_k=10, top_p=.92,
                    max_length=1000, pad_token=tokenizer.eos_token_id):
        full_msg = self.model.generate(bot_input_ids, do_sample=True,
                                                top_k=top_k, top_p=top_p, 
                                                max_length=max_length, pad_token_id=self.tokenizer.eos_token_id)
        msg = self.to_data(full_msg.detach()[0])[bot_input_ids.shape[-1]:]
        return msg

    def start_conversation(self, input_fa):

        # The first use of PersonaGPT is to do personalized dialog generation. Use the following loop to interact with the model.
        # converse for 8 turns
        dialog_hx = []
        input_en = self.translator_fa.translate(input_fa)
        # encode the user input
        user_inp = self.tokenizer.encode(input_en + self.tokenizer.eos_token)
        # append to the chat history
        dialog_hx.append(user_inp)
            
        # generated a response while limiting the total chat history to 1000 tokens, 
        bot_input_ids = self.to_var([self.personas + self.flatten(dialog_hx)]).long()
        msg = self.generate_next(bot_input_ids)
        dialog_hx.append(msg)
        response_en = "Bot: {}".format(self.tokenizer.decode(msg, skip_special_tokens=True))
        response_fa = self.translator_en.translate(response_en)
        
        return response_fa


if __name__ == "__main__":
    try:
        ConversationService()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logdebug(e)
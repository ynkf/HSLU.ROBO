from config import api_key
import requests
import rospy

class GeminiAdapter:
    __gemini_url = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=' + api_key

    def __init__(self, initMessage):

        self.__init_prompt = initMessage

    def toPythonRequest(self, text):
        return {
            "contents": [{
                "parts": [{"text": text}]
            }]
        }

    def request(self, text):
        json = self.toPythonRequest(f'{self.__init_prompt} {text}')
        rospy.loginfo(f'gemini request body: {json}')
        res = requests.post(self.__gemini_url, json=json)

        if res.status_code != 200:
            raise Exception("Request to Gemini failed: "+ res.text)

        return res.json()['candidates'][0]['content']['parts'][0]['text']

if __name__ == '__main__':
    GeminiAdapter()
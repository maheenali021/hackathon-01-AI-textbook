# Hugging Face Space for RAG Agent API
import os
from gradio import Interface, components
import requests
import json

def query_agent(query: str) -> str:
    '''Function to query the RAG agent'''
    # Use environment variables for API configuration
    api_url = os.getenv('API_URL', 'http://localhost:8001/api/v1/chat')
    
    payload = {
        'query': query
    }
    
    try:
        response = requests.post(
            api_url,
            json=payload,
            headers={'Content-Type': 'application/json'}
        )
        
        if response.status_code == 200:
            result = response.json()
            return result.get('response', 'No response received')
        else:
            return f'Error: {response.status_code} - {response.text}'
    except Exception as e:
        return f'Error occurred: {str(e)}'

# Create Gradio interface
interface = Interface(
    fn=query_agent,
    inputs=components.Textbox(lines=3, placeholder='Enter your question here...', label='Question'),
    outputs=components.Textbox(label='Response'),
    title='AI Robotics Textbook Assistant',
    description='Ask questions about the AI Robotics textbook and get answers based on the content.',
    examples=[
        ['What is this book about?'],
        ['Explain the ROS2 architecture basics'],
        ['How do I set up ROS2 Humble?']
    ]
)

# Launch the interface
if __name__ == '__main__':
    interface.launch(server_name='0.0.0.0', server_port=7860)


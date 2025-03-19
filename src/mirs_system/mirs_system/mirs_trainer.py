import torch
import torch.nn as nn
import torch.optim as optim
from transformers import BertModel, BertTokenizer
from torchvision import models, transforms
import numpy as np
import random

# 1. Image Feature Extractor (Vision Transformer or CNN)
class ImageEncoder(nn.Module):
    def __init__(self, output_dim=256):
        super(ImageEncoder, self).__init__()
        # Using a pretrained ResNet as a feature extractor
        self.resnet = models.resnet18(pretrained=True)
        self.resnet.fc = nn.Linear(self.resnet.fc.in_features, output_dim)

    def forward(self, x):
        return self.resnet(x)

# 2. Text Feature Extractor (BERT)
class TextEncoder(nn.Module):
    def __init__(self, pretrained_model='bert-base-uncased', output_dim=256):
        super(TextEncoder, self).__init__()
        self.bert = BertModel.from_pretrained(pretrained_model)
        self.fc = nn.Linear(self.bert.config.hidden_size, output_dim)

    def forward(self, x):
        # Tokenize and get the embeddings from BERT
        outputs = self.bert(input_ids=x['input_ids'], attention_mask=x['attention_mask'])
        return self.fc(outputs.pooler_output)

# 3. Combined Transformer Model
class TransformerModel(nn.Module):
    def __init__(self, image_encoder, text_encoder, hidden_dim=512):
        super(TransformerModel, self).__init__()
        self.image_encoder = image_encoder
        self.text_encoder = text_encoder
        
        # Transformer Layers
        self.transformer = nn.Transformer(d_model=hidden_dim, num_encoder_layers=6, num_decoder_layers=6)

        # Output layer for predicting robot state
        self.fc = nn.Linear(hidden_dim, 7) # 7 for robot state (position, orientation, etc.)

    def forward(self, image_seq, text_seq):
        # Process image sequence through the image encoder
        image_features = [self.image_encoder(img) for img in image_seq]

        # Process text sequence through the text encoder
        text_features = [self.text_encoder(text) for text in text_seq]

        # Combine both features
        combined_features = torch.cat([torch.stack(image_features), torch.stack(text_features)], dim=0)

        # Transformer expects the input to be (seq_len, batch, feature_size)
        transformer_input = combined_features.permute(1, 0, 2) # seq_len, batch, feature_size

        # Pass through the transformer layers
        transformer_output = self.transformer(transformer_input, transformer_input)

        # Get robot state output
        robot_state = self.fc(transformer_output[-1, :, :]) # Only using the last output

        return robot_state

# 4. Reinforcement Learning Environment (Pass the Predicted Pose to Robot & get the status from the robot)

class RobotEnv:
    def __init__(self):
        self.state = np.zeros(6) # 6 for robot pose (position, orientation)
        self.goal_state = np.ones(6) # Define a goal pose for the task

    def get_robot_state(self):
        #Subscribe to robot state
        return self.robot_state
    
    def publish_robot_task(self,msg):
        pass

    def taskstatus_subscriber(self,msg):
        self.task_status = msg
        self.update_reward()

    def robotstate_subscriber(self,msg):
        self.robot_state = msg

    def update_reward(self):
        # If the pose can not be achielded provide negative reward
        # If the pose can be achielded at single orientaion less positve reward
        # If the pose can be achielded at multiple orientaion more positve rewardss
        # Here we would integrate robot's dynamics and physics

        if self.task_status == "NOT POSSIBLE":
            self.reward = -10
        elif self.task_status == "SINGLE ORIENTATION":
            self.reward = 10
        elif self.task_status == "MULTIPLE ORIENTATION":
            self.reward = 100
        else: 
            self.reward = -np.linalg.norm(self.state - self.goal_state) # Reward based on proximity to goal
        
    def get_reward(self):
        return self.reward

    def step(self, action):
        # Simple placeholder: update state with action and calculate reward
        self.publish_robot_task(action)
        # Wait till you get the response from the robot
        self.state = self.get_robot_state() # Update state based on action (simplified)
        reward = self.get_reward()
        done = np.allclose(self.state, self.goal_state, atol=0.1)
        return self.state, reward, done

    def reset(self):
        self.state = np.zeros(6)
        return self.state

# 5. Reinforcement Learning with Policy Gradient or DQN
class ReinforcementLearningAgent:
    def __init__(self, model, env, learning_rate=1e-4, gamma=0.99):
        self.model = model
        self.env = env
        self.optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        self.gamma = gamma

    def train(self, num_episodes=1000):
        for episode in range(num_episodes):
            state = self.env.reset()
            total_reward = 0
            done = False
            
            while not done:
                # Random action for simplicity (Replace with a policy network)
                # action = np.random.uniform(-0.1, 0.1, size=(7,))
                action = self.model.predict()
                
                # Take action in the environment
                next_state, reward, done = self.env.step(action)
                total_reward += reward

                # Compute the loss (for simplicity, we'll just use the reward as the target)
                # A more complex policy gradient or Q-learning would be used here
                predicted_state = torch.tensor(state, dtype=torch.float32).unsqueeze(0) # Add batch dimension
                target_state = torch.tensor(next_state, dtype=torch.float32).unsqueeze(0)
                loss = torch.nn.functional.mse_loss(predicted_state, target_state)

                # Backpropagation
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()

                state = next_state

            print(f"Episode {episode}, Total Reward: {total_reward}")

# 6. Putting It All Together
if __name__ == "__main__":
    # Initialize components
    image_encoder = ImageEncoder()
    text_encoder = TextEncoder()
    transformer_model = TransformerModel(image_encoder, text_encoder)

    env = RobotEnv()
    agent = ReinforcementLearningAgent(transformer_model, env)

    # Start training
    agent.train(num_episodes=1000)

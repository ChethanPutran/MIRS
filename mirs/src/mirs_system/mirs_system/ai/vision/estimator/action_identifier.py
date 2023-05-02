from mirs_system.ai.vision.estimator.models.action_identifier import model


class ActionIdentifier:
    def __init__(self) -> None:
        self.model = model.Model()
    def identify_action(self,video):
        return self.model.predict(video) 
 
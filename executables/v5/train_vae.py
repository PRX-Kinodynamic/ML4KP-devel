from models.vae import VAE


class Runner:
    def __init__(self):
        self.vae = VAE()
        
    def train(self):
        self.vae.model.train()
        
    def test(self):
        self.vae.model.eval()

if __name__ == "__main__":
    runner = Runner()
    runner.train()
    runner.test()
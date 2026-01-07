"""
Script to add test documentation to the vector database for immediate testing
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from src.embedding.generator import generate_embeddings
from src.storage.qdrant_client import store_embeddings
from src.utils.config import load_config
from src.utils.logger import setup_logger

logger = setup_logger()
config = load_config()

def add_test_documents():
    """Add some test documentation to the vector database"""
    logger.info("Adding test documentation to vector database...")

    # Sample documentation content that matches what would be on the site
    test_docs = [
        {
            "content": "Physical AI is a field that combines artificial intelligence with physical systems and robotics. It involves creating AI that can interact with the physical world through sensors, actuators, and robotic systems.",
            "metadata": {
                "url": "https://hackathon-1-chi-livid.vercel.app/docs/intro",
                "title": "Introduction to Physical AI",
                "section": "Getting Started"
            }
        },
        {
            "content": "Robotic systems use sensors to perceive their environment and actuators to interact with it. Common sensors include cameras, LIDAR, accelerometers, and gyroscopes. Actuators include motors, servos, and pneumatic systems.",
            "metadata": {
                "url": "https://hackathon-1-chi-livid.vercel.app/docs/robotics",
                "title": "Robotic Systems Fundamentals",
                "section": "Core Concepts"
            }
        },
        {
            "content": "Machine learning in robotics involves training models to control robotic systems. This includes path planning, object recognition, manipulation, and navigation. Deep learning techniques are often used for perception tasks.",
            "metadata": {
                "url": "https://hackathon-1-chi-livid.vercel.app/docs/ml-robotics",
                "title": "Machine Learning for Robotics",
                "section": "Advanced Topics"
            }
        },
        {
            "content": "Embodied AI refers to artificial intelligence that is integrated into physical systems. Unlike traditional AI that processes data in isolation, embodied AI learns and operates through interaction with the physical environment.",
            "metadata": {
                "url": "https://hackathon-1-chi-livid.vercel.app/docs/embodied-ai",
                "title": "Embodied Artificial Intelligence",
                "section": "Core Concepts"
            }
        },
        {
            "content": "Computer vision in robotics enables robots to interpret visual information from cameras and other imaging systems. This includes object detection, tracking, recognition, and scene understanding.",
            "metadata": {
                "url": "https://hackathon-1-chi-livid.vercel.app/docs/computer-vision",
                "title": "Computer Vision for Robotics",
                "section": "Perception"
            }
        },
        {
            "content": "Motion planning algorithms determine how a robot should move from one location to another while avoiding obstacles. Common approaches include A* search, RRT (Rapidly-exploring Random Trees), and potential fields.",
            "metadata": {
                "url": "https://hackathon-1-chi-livid.vercel.app/docs/motion-planning",
                "title": "Motion Planning Algorithms",
                "section": "Navigation"
            }
        }
    ]

    try:
        logger.info(f"Generating embeddings for {len(test_docs)} test documents...")

        # Extract just the content for embedding generation
        contents = [doc["content"] for doc in test_docs]
        embeddings = generate_embeddings(test_docs)  # Pass full docs with content field

        logger.info(f"Successfully generated {len(embeddings)} embeddings")

        logger.info("Storing embeddings in Qdrant...")
        store_embeddings(test_docs, embeddings, config)

        logger.info("Test documentation successfully added to vector database!")
        logger.info("The RAG chatbot should now be able to answer questions about Physical AI, robotics, computer vision, motion planning, and embodied AI.")

        return True

    except Exception as e:
        logger.error(f"Error adding test documentation: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = add_test_documents()
    if success:
        print("\n✓ Test documentation added successfully!")
        print("The chatbot should now be able to answer questions about the documentation.")
    else:
        print("\n✗ Failed to add test documentation")
        sys.exit(1)
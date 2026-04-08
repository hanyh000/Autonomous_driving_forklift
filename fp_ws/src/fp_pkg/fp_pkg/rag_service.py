import chromadb
from sentence_transformers import SentenceTransformer
from typing import List
from langchain_text_splitters import RecursiveCharacterTextSplitter

class RAGService:
    def __init__(self, collection_name="robot_knowledge"):
        self.client = chromadb.PersistentClient(path="/home/dev/fp_ws/rag_db")
        self.encoder = SentenceTransformer('snunlp/KR-SBERT-V40K-klueNLI-augSTS')  # 한국어 모델
        self.collection = self.client.get_or_create_collection(
            name=collection_name,
            metadata={"hnsw:space": "cosine"}
        )

        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=500,    # 500자 단위로 자름
            chunk_overlap=50,  # 문맥 유지를 위해 50자씩 겹침
            length_function=len,
        )

    def add_documents_with_chunking(self, docs: List[dict]):
        all_chunks = []
        
        for d in docs:
            # 문서를 작은 조각으로 분할
            chunks = self.text_splitter.split_text(d['text'])
            
            for i, chunk in enumerate(chunks):
                all_chunks.append({
                    'id': f"{d['id']}_{i}", # ID 중복 방지
                    'text': chunk,
                    'metadata': d.get('metadata', {})
                })
                
        self.add_documents(all_chunks)

    def add_documents(self, docs: List[dict]):
        """docs: [{'id': str, 'text': str, 'metadata': dict}]"""
        ids = [d['id'] for d in docs]
        texts = [d['text'] for d in docs]
        embeddings = self.encoder.encode(texts).tolist()
        metadatas = [d.get('metadata', {}) for d in docs]

        self.collection.upsert(
            ids=ids,
            embeddings=embeddings,
            documents=texts,
            metadatas=metadatas
        )

    def retrieve(self, query: str, top_k: int = 3) -> str:
        embedding = self.encoder.encode([query]).tolist()
        results = self.collection.query(
            query_embeddings=embedding,
            n_results=top_k
        )
        print(f"[RAG] 유사도 distances: {results['distances']}")
        docs = results['documents'][0]
        return "\n".join([f"- {d}" for d in docs])
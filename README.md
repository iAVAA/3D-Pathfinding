# 3D Pathfinding Visualizer

Un'applicazione web interattiva per visualizzare e confrontare algoritmi di pathfinding in uno spazio 3D. 
Il progetto permette di creare ostacoli personalizzati, impostare punti di partenza e di arrivo, ed eseguire gli algoritmi per trovare i percorsi migliori.

![Screenshot dell'Applicazione](placeholder_image.png)

## 📋 Funzionalità
- **Visualizzazione 3D Interattiva:** Naviga nello spazio, ruota la visuale ed esamina le rotte e gli ostacoli in 3D.
- **Griglia Personalizzabile:** Modifica le dimensioni del mondo (es. 10x10x10) e inserisci dinamicamente blocchi come ostacoli.
- **Algoritmi Multipli:**
  - **A***: L'algoritmo standard e bilanciato.
  - **Theta***: A* ottimizzato con Line of Sight per traiettorie più naturali.
  - **D* Lite**: Variante dinamica che si adatta alle modifiche, ideale per ambienti variabili.
  - **JPS (Jump Point Search)**: Ottimizzazione per grid uniformi.
- **Comparazione:** Confronta visivamente i percorsi e consulta i tempi di esecuzione e la quantità di passi in una comoda tabella.
- **Tema Premium:** Interfaccia in stile "cyber/neon" scura, focalizzata sui dati e facile da leggere.

## 🏗 Struttura del Progetto

Il progetto è stato refactorizzato per essere modulare e mantenibile:

```text
3D-Pathfinding/
├── README.md                 # Questo file
├── Dockerfile                # Configurazione per Docker
├── docker-compose.yaml       # Orchestrazione container
├── nginx.conf                # Configurazione server Nginx
├── pages/
│   └── index.html            # Entry point HTML dell'app
├── css/
│   └── style.css             # Fogli di stile isolati
└── js/
    ├── app.js                # Logica principale UI e Three.js
    ├── core/
    │   ├── utils.js          # Strutture dati condivise (MinHeap, distanze)
    │   └── octree.js         # Classe per indicizzare lo spazio 3D e gestire collisioni
    └── algorithms/
        ├── astar.js          # Implementazione A*
        ├── thetastar.js      # Implementazione Theta*
        ├── dstar.js          # Implementazione D* Lite
        └── jps.js            # Implementazione Jump Point Search (3D)
```

## 🚀 Come eseguire il progetto

### Con Docker (Consigliato)

1. Assicurati di avere [Docker](https://www.docker.com/) e `docker-compose` installati sul tuo sistema.
2. Apri il terminale nella cartella root del progetto (`3D-Pathfinding`).
3. Esegui il comando:
   ```bash
   docker-compose up --build -d
   ```
4. Apri il browser e vai all'indirizzo: **http://localhost:80** (o la porta configurata in docker-compose).
5. Per fermare il progetto:
   ```bash
   docker-compose down
   ```

### Senza Docker (In locale)

Poiché il progetto utilizza moduli ES e file esterni, serve un web server locale per aggirare le restrizioni CORS sui protocolli `file://`.
Puoi utilizzare estensioni di VSCode (es. **Live Server**) o un server rapido via terminale:

**Con Python:**
```bash
python3 -m http.server 8000
# Poi apri http://localhost:8000/pages/index.html
```

**Con Node.js (http-server):**
```bash
npx http-server -p 8000
# Poi apri http://localhost:8000/pages/index.html
```

## 📸 Immagini / Screenshots

*Di seguito puoi inserire gli screen della tua applicazione:*

1. **Interfaccia Principale:**
   ![Main UI](placeholder_ui.png)
   
2. **Confronto tra algoritmi:**
   ![Confronto](placeholder_comparison.png)

3. **Dettaglio di un percorso generato:**
   ![Dettaglio Percorso](placeholder_path.png)

## 🛠 Tecnologie Utilizzate
- **HTML5 / CSS3** (Nessun framework CSS)
- **Vanilla JavaScript** (ES6+)
- **[Three.js](https://threejs.org/)** (R128 per il rendering WebGL)
- **Nginx & Docker** (Per il deployment)

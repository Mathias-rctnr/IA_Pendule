# IA_Pendule (CSFML)

<img width="3024" height="2292" alt="image" src="https://github.com/user-attachments/assets/4eddbdb4-e7d2-42cf-b10a-951614d3de31" />

Petit projet perso pour tester un pendule contrôlé par un algorithme génétique.  
Le but est simple : faire apprendre à un réseau de neurones à stabiliser la masse vers le haut, en jouant uniquement sur la vitesse de la base.

## Ce que fait le projet
- Simulation d’un pendule à base mobile en 2D
- Un réseau de neurones choisit la vitesse de la base
- Un algorithme génétique fait évoluer les réseaux (évaluation → sélection → mutations)
- Visualisation en temps réel + mode rapide pour entraîner sans afficher

## Commandes
- **Clic sur le bouton** : démarrer / arrêter le GA  
- **F** : basculer entre mode rapide (FAST) et mode affichage (DISPLAY)

## Ce qu’il faut savoir
- L’entraînement est long : les bonnes générations commencent généralement vers **1500–2000** (ça dépend des paramètres).
- Tous les paramètres sont ajustables (récompense, mutations, physique).  
- C’est un projet perso, donc le code évolue au fil des tests.

## Compilation (macOS)
```bash
gcc main.c pendulum.c ga.c -o pendule \
  -I/opt/homebrew/include \
  -L/opt/homebrew/lib \
  -lcsfml-graphics -lcsfml-window -lcsfml-system -lcsfml-audio -lm -pthread

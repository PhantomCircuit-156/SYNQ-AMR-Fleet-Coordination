"""Start the SYNQ local dashboard on http://127.0.0.1:8000."""
from pathlib import Path
import sys
import uvicorn
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
uvicorn.run("dashboard.app:app", host="127.0.0.1", port=8000, reload=False)
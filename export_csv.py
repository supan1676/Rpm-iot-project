"""
Export Patient Vitals from Supabase to CSV
Usage:
    python export_csv.py
"""

import urllib.request
import os
from datetime import datetime

SUPABASE_URL = "https://iwparjibxhqqwatapfbz.supabase.co/rest/v1/patient_vitals?select=*&order=created_at.desc"
SUPABASE_KEY = "sb_publishable_6CfIvqtBVWjqNMv1v__Taw_h7zN8ABm"
OUTPUT_FILE = "patient_vitals.csv"

def export_csv():
    print(f"Connecting to Supabase (iwparjibxhqqwatapfbz)...")
    headers = {
        "apikey": SUPABASE_KEY,
        "Authorization": f"Bearer {SUPABASE_KEY}",
        "Accept": "text/csv"
    }

    req = urllib.request.Request(SUPABASE_URL, headers=headers)
    try:
        with urllib.request.urlopen(req) as resp:
            if resp.status == 200:
                content = resp.read().decode('utf-8')
                with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
                    f.write(content)
                line_count = len(content.strip().split("\n")) - 1
                print(f"[SUCCESS] Exported {line_count} record(s) to '{OUTPUT_FILE}'.")
            else:
                print(f"[ERROR] Failed with HTTP Status: {resp.status}")
    except Exception as e:
        print(f"[ERROR] Fetch failed: {e}")

if __name__ == "__main__":
    export_csv()

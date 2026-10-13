import sqlite3
import sys
from robotic_arm.adapters.app_data_directory import default_data_directory

def main():
    shift_amount = 75.0
    db_path = default_data_directory() / "profiles.db"
    
    if not db_path.exists():
        print(f"Error: No se encontro la base de datos en {db_path}")
        sys.exit(1)
        
    print(f"Conectando a la base de datos en {db_path}...")
    conn = sqlite3.connect(db_path)
    
    try:
        cursor = conn.cursor()
        cursor.execute("SELECT position, arm_x, arm_y FROM reference_points")
        points = cursor.fetchall()
        print("Antes de actualizar:")
        for p in points:
            print(f"  Punto {p[0]}: X={p[1]}, Y={p[2]}")
            
        # Sumar 75mm al X de todos los puntos de calibración
        cursor.execute("UPDATE reference_points SET arm_x = arm_x + ?", (shift_amount,))
        conn.commit()
        
        cursor.execute("SELECT position, arm_x, arm_y FROM reference_points")
        points = cursor.fetchall()
        print(f"\nDespués de añadir {shift_amount}mm al eje X:")
        for p in points:
            print(f"  Punto {p[0]}: X={p[1]}, Y={p[2]}")
            
        print("\n¡Éxito! Los puntos de calibración han sido desplazados hacia adelante.")
    except Exception as e:
        print(f"Error al actualizar la base de datos: {e}")
        conn.rollback()
    finally:
        conn.close()

if __name__ == '__main__':
    main()

"""
Deadlock Predictor Training Script
===================================

สคริปต์สำหรับเทรน ML Model เพื่อทำนาย Deadlock จากข้อมูล logs

การใช้งาน:
    python scripts/train_deadlock_model.py
    python scripts/train_deadlock_model.py --logs-dir logs/20251220_003401
    python scripts/train_deadlock_model.py --all-logs
"""

import os
import re
import glob
import argparse
import pickle
from datetime import datetime
from collections import defaultdict
from typing import List, Dict, Tuple, Optional

import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import (
    classification_report, 
    confusion_matrix, 
    accuracy_score,
    precision_score,
    recall_score,
    f1_score,
    roc_auc_score
)
import joblib


# ===========================
# Configuration
# ===========================

LOGS_BASE_DIR = "logs"
MODELS_DIR = "models"
MODEL_OUTPUT_PATH = os.path.join(MODELS_DIR, "deadlock_predictor.pkl")

# Regex patterns สำหรับ parse log
MOVE_PATTERN = re.compile(
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \| MOVE \[(\d+), (\d+)\] -> \[(\d+), (\d+)\] \| STATE=(\w+) \| MODE=(\w+)'
)
BLOCKED_PATTERN = re.compile(
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \| BLOCKED \[(\d+), (\d+)\] -> \[(\d+), (\d+)\] \| WAIT=(\d+)'
)
YIELD_PATTERN = re.compile(
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \| YIELD to R(\d+) -> \[(\d+), (\d+)\]'
)
RETREAT_PATTERN = re.compile(
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \| RETREAT -> \[(\d+), (\d+)\]'
)
EMERGENCY_PATTERN = re.compile(
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \| EMERGENCY MOVE -> \[(\d+), (\d+)\]'
)
PICKUP_PATTERN = re.compile(
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \| PICKUP (\w+) @ \[(\d+), (\d+)\]'
)
DROPOFF_PATTERN = re.compile(
    r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) \| DROPOFF (\w+) @ \[(\d+), (\d+)\]'
)


# ===========================
# Data Parsing
# ===========================

class LogParser:
    """Parse robot log files"""
    
    def __init__(self):
        self.events = []
        
    def parse_file(self, filepath: str, robot_name: str) -> List[Dict]:
        """Parse single log file"""
        events = []
        
        with open(filepath, 'r', encoding='utf-8') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line:
                    continue
                
                event = self._parse_line(line, robot_name, line_num)
                if event:
                    events.append(event)
        
        return events
    
    def _parse_line(self, line: str, robot_name: str, line_num: int) -> Optional[Dict]:
        """Parse single log line"""
        
        # MOVE event
        match = MOVE_PATTERN.match(line)
        if match:
            return {
                'timestamp': match.group(1),
                'robot': robot_name,
                'event_type': 'MOVE',
                'from_row': int(match.group(2)),
                'from_col': int(match.group(3)),
                'to_row': int(match.group(4)),
                'to_col': int(match.group(5)),
                'state': match.group(6),
                'mode': match.group(7),
                'wait': 0,
                'is_blocked': False,
                'is_deadlock': False
            }
        
        # BLOCKED event
        match = BLOCKED_PATTERN.match(line)
        if match:
            return {
                'timestamp': match.group(1),
                'robot': robot_name,
                'event_type': 'BLOCKED',
                'from_row': int(match.group(2)),
                'from_col': int(match.group(3)),
                'to_row': int(match.group(4)),
                'to_col': int(match.group(5)),
                'state': 'UNKNOWN',
                'mode': 'UNKNOWN',
                'wait': int(match.group(6)),
                'is_blocked': True,
                'is_deadlock': False
            }
        
        # YIELD event (indicates potential deadlock situation)
        match = YIELD_PATTERN.match(line)
        if match:
            return {
                'timestamp': match.group(1),
                'robot': robot_name,
                'event_type': 'YIELD',
                'from_row': 0,
                'from_col': 0,
                'to_row': int(match.group(3)),
                'to_col': int(match.group(4)),
                'state': 'YIELDING',
                'mode': 'YIELDING',
                'wait': 0,
                'is_blocked': True,
                'is_deadlock': True  # YIELD indicates deadlock-like situation
            }
        
        # RETREAT event
        match = RETREAT_PATTERN.match(line)
        if match:
            return {
                'timestamp': match.group(1),
                'robot': robot_name,
                'event_type': 'RETREAT',
                'from_row': 0,
                'from_col': 0,
                'to_row': int(match.group(2)),
                'to_col': int(match.group(3)),
                'state': 'RETREAT',
                'mode': 'RETREAT',
                'wait': 0,
                'is_blocked': True,
                'is_deadlock': True
            }
        
        # EMERGENCY event
        match = EMERGENCY_PATTERN.match(line)
        if match:
            return {
                'timestamp': match.group(1),
                'robot': robot_name,
                'event_type': 'EMERGENCY',
                'from_row': 0,
                'from_col': 0,
                'to_row': int(match.group(2)),
                'to_col': int(match.group(3)),
                'state': 'EMERGENCY',
                'mode': 'FORCED',
                'wait': 0,
                'is_blocked': True,
                'is_deadlock': True
            }
        
        return None
    
    def parse_directory(self, log_dir: str) -> pd.DataFrame:
        """Parse all robot logs in a directory"""
        all_events = []
        
        log_files = glob.glob(os.path.join(log_dir, "R*.log"))
        
        for log_file in log_files:
            robot_name = os.path.basename(log_file).replace('.log', '')
            events = self.parse_file(log_file, robot_name)
            all_events.extend(events)
        
        if not all_events:
            return pd.DataFrame()
        
        df = pd.DataFrame(all_events)
        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df = df.sort_values('timestamp').reset_index(drop=True)
        
        return df


# ===========================
# Feature Engineering
# ===========================

class FeatureEngineer:
    """สร้าง features สำหรับ ML model"""
    
    def __init__(self, window_size: int = 5):
        self.window_size = window_size
    
    def create_features(self, df: pd.DataFrame) -> pd.DataFrame:
        """สร้าง features จาก raw events"""
        
        if df.empty:
            return pd.DataFrame()
        
        features = []
        
        for robot in df['robot'].unique():
            robot_df = df[df['robot'] == robot].copy()
            robot_features = self._create_robot_features(robot_df)
            features.extend(robot_features)
        
        return pd.DataFrame(features)
    
    def _create_robot_features(self, robot_df: pd.DataFrame) -> List[Dict]:
        """สร้าง features สำหรับ robot เดียว"""
        features = []
        
        for i in range(len(robot_df)):
            row = robot_df.iloc[i]
            
            # Skip invalid rows
            if row['event_type'] not in ['MOVE', 'BLOCKED']:
                continue
            
            feature = {
                # Basic position features
                'from_row': row['from_row'],
                'from_col': row['from_col'],
                'to_row': row['to_row'],
                'to_col': row['to_col'],
                
                # Movement direction
                'dir_row': row['to_row'] - row['from_row'],
                'dir_col': row['to_col'] - row['from_col'],
                
                # Current state
                'wait': row['wait'],
                
                # State encoding
                'state_TO_PICKUP': 1 if row['state'] == 'TO_PICKUP' else 0,
                'state_TO_DROPOFF': 1 if row['state'] == 'TO_DROPOFF' else 0,
                'state_HOME': 1 if row['state'] == 'HOME' else 0,
                'state_IDLE': 1 if row['state'] == 'IDLE' else 0,
                'state_EVACUATING': 1 if row['state'] == 'EVACUATING' else 0,
                
                # Mode encoding
                'mode_NORMAL': 1 if row['mode'] == 'NORMAL' else 0,
                'mode_YIELDING': 1 if row['mode'] == 'YIELDING' else 0,
                'mode_FORCED': 1 if row['mode'] == 'FORCED' else 0,
                
                # Historical features (look back)
                'recent_blocks': self._count_recent_events(robot_df, i, 'BLOCKED'),
                'recent_moves': self._count_recent_events(robot_df, i, 'MOVE'),
                
                # Target
                'is_deadlock': row['is_deadlock'] or self._will_deadlock_soon(robot_df, i)
            }
            
            features.append(feature)
        
        return features
    
    def _count_recent_events(self, df: pd.DataFrame, current_idx: int, event_type: str) -> int:
        """นับ events ล่าสุด"""
        start_idx = max(0, current_idx - self.window_size)
        window = df.iloc[start_idx:current_idx]
        return len(window[window['event_type'] == event_type])
    
    def _will_deadlock_soon(self, df: pd.DataFrame, current_idx: int) -> bool:
        """ตรวจสอบว่าจะเกิด deadlock ใน window ถัดไปหรือไม่"""
        end_idx = min(len(df), current_idx + self.window_size)
        future = df.iloc[current_idx:end_idx]
        
        deadlock_events = ['YIELD', 'RETREAT', 'EMERGENCY']
        return any(future['event_type'].isin(deadlock_events))


# ===========================
# Model Training
# ===========================

class DeadlockModelTrainer:
    """Train deadlock prediction model"""
    
    def __init__(self):
        self.models = {
            'RandomForest': RandomForestClassifier(
                n_estimators=100,
                max_depth=10,
                min_samples_split=5,
                random_state=42,
                n_jobs=-1
            ),
            'GradientBoosting': GradientBoostingClassifier(
                n_estimators=100,
                max_depth=5,
                learning_rate=0.1,
                random_state=42
            ),
            'LogisticRegression': LogisticRegression(
                max_iter=1000,
                random_state=42
            )
        }
        self.scaler = StandardScaler()
        self.best_model = None
        self.best_model_name = None
        self.feature_columns = None
    
    def prepare_data(self, df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
        """เตรียมข้อมูลสำหรับ training"""
        
        # Feature columns (exclude target)
        self.feature_columns = [col for col in df.columns if col != 'is_deadlock']
        
        X = df[self.feature_columns].values
        y = df['is_deadlock'].astype(int).values
        
        return X, y
    
    def train(self, X: np.ndarray, y: np.ndarray) -> Dict:
        """Train และเลือก model ที่ดีที่สุด"""
        
        # Split data
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=42, stratify=y
        )
        
        # Scale features
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_test_scaled = self.scaler.transform(X_test)
        
        results = {}
        
        for name, model in self.models.items():
            print(f"\n🔄 Training {name}...")
            
            # Train
            if name == 'LogisticRegression':
                model.fit(X_train_scaled, y_train)
                y_pred = model.predict(X_test_scaled)
                y_proba = model.predict_proba(X_test_scaled)[:, 1]
            else:
                model.fit(X_train, y_train)
                y_pred = model.predict(X_test)
                y_proba = model.predict_proba(X_test)[:, 1]
            
            # Evaluate
            accuracy = accuracy_score(y_test, y_pred)
            precision = precision_score(y_test, y_pred, zero_division=0)
            recall = recall_score(y_test, y_pred, zero_division=0)
            f1 = f1_score(y_test, y_pred, zero_division=0)
            
            try:
                roc_auc = roc_auc_score(y_test, y_proba)
            except:
                roc_auc = 0.0
            
            results[name] = {
                'model': model,
                'accuracy': accuracy,
                'precision': precision,
                'recall': recall,
                'f1': f1,
                'roc_auc': roc_auc,
                'y_test': y_test,
                'y_pred': y_pred
            }
            
            print(f"   ✅ Accuracy: {accuracy:.4f}")
            print(f"   ✅ Precision: {precision:.4f}")
            print(f"   ✅ Recall: {recall:.4f}")
            print(f"   ✅ F1 Score: {f1:.4f}")
            print(f"   ✅ ROC-AUC: {roc_auc:.4f}")
        
        # Select best model (by F1 score)
        best_name = max(results, key=lambda x: results[x]['f1'])
        self.best_model = results[best_name]['model']
        self.best_model_name = best_name
        
        print(f"\n🏆 Best Model: {best_name} (F1: {results[best_name]['f1']:.4f})")
        
        return results
    
    def save_model(self, output_path: str):
        """Save trained model"""
        model_data = {
            'model': self.best_model,
            'model_name': self.best_model_name,
            'scaler': self.scaler,
            'feature_columns': self.feature_columns,
            'trained_at': datetime.now().isoformat()
        }
        
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        joblib.dump(model_data, output_path)
        
        print(f"\n💾 Model saved to: {output_path}")
    
    def cross_validate(self, X: np.ndarray, y: np.ndarray, cv: int = 5) -> Dict:
        """Perform cross-validation"""
        print(f"\n📊 Performing {cv}-fold cross-validation...")
        
        cv_results = {}
        
        for name, model in self.models.items():
            scores = cross_val_score(model, X, y, cv=cv, scoring='f1')
            cv_results[name] = {
                'mean': scores.mean(),
                'std': scores.std(),
                'scores': scores
            }
            print(f"   {name}: {scores.mean():.4f} (+/- {scores.std()*2:.4f})")
        
        return cv_results


# ===========================
# Main Function
# ===========================

def find_all_log_dirs(base_dir: str) -> List[str]:
    """หา log directories ทั้งหมด"""
    log_dirs = []
    
    if os.path.exists(base_dir):
        for entry in os.listdir(base_dir):
            full_path = os.path.join(base_dir, entry)
            if os.path.isdir(full_path):
                log_dirs.append(full_path)
    
    return sorted(log_dirs)


def main():
    parser = argparse.ArgumentParser(description='Train Deadlock Predictor Model')
    parser.add_argument('--logs-dir', type=str, help='Specific log directory to use')
    parser.add_argument('--all-logs', action='store_true', help='Use all log directories')
    parser.add_argument('--output', type=str, default=MODEL_OUTPUT_PATH, help='Output model path')
    parser.add_argument('--cv', type=int, default=5, help='Cross-validation folds')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("🤖 Deadlock Predictor Training")
    print("=" * 60)
    
    # Determine log directories
    if args.logs_dir:
        log_dirs = [args.logs_dir]
    elif args.all_logs:
        log_dirs = find_all_log_dirs(LOGS_BASE_DIR)
    else:
        # Use most recent log directory
        all_dirs = find_all_log_dirs(LOGS_BASE_DIR)
        log_dirs = [all_dirs[-1]] if all_dirs else []
    
    if not log_dirs:
        print("❌ No log directories found!")
        return
    
    print(f"\n📁 Using log directories: {log_dirs}")
    
    # Parse logs
    print("\n📖 Parsing log files...")
    log_parser = LogParser()
    
    all_events = []
    for log_dir in log_dirs:
        df = log_parser.parse_directory(log_dir)
        if not df.empty:
            all_events.append(df)
            print(f"   ✅ {log_dir}: {len(df)} events")
    
    if not all_events:
        print("❌ No events found in logs!")
        return
    
    events_df = pd.concat(all_events, ignore_index=True)
    print(f"\n📊 Total events: {len(events_df)}")
    
    # Feature engineering
    print("\n🔧 Creating features...")
    feature_engineer = FeatureEngineer(window_size=5)
    features_df = feature_engineer.create_features(events_df)
    
    if features_df.empty:
        print("❌ No features created!")
        return
    
    print(f"   ✅ Created {len(features_df)} samples with {len(features_df.columns)-1} features")
    
    # Check class balance
    deadlock_count = features_df['is_deadlock'].sum()
    total_count = len(features_df)
    print(f"\n📈 Class distribution:")
    print(f"   Normal: {total_count - deadlock_count} ({(1 - deadlock_count/total_count)*100:.1f}%)")
    print(f"   Deadlock: {deadlock_count} ({deadlock_count/total_count*100:.1f}%)")
    
    # Handle class imbalance
    if deadlock_count < 10:
        print("\n⚠️ Warning: Very few deadlock samples. Consider collecting more data.")
        
        # Synthetic data augmentation for deadlock cases
        print("   Augmenting deadlock samples...")
        deadlock_samples = features_df[features_df['is_deadlock'] == True]
        
        if len(deadlock_samples) > 0:
            # Simple augmentation: duplicate with slight noise
            augmented = []
            for _ in range(5):  # 5x augmentation
                for _, row in deadlock_samples.iterrows():
                    new_row = row.copy()
                    # Add small noise to numeric features
                    new_row['wait'] = max(0, new_row['wait'] + np.random.randint(-1, 2))
                    new_row['recent_blocks'] = max(0, new_row['recent_blocks'] + np.random.randint(-1, 2))
                    augmented.append(new_row)
            
            if augmented:
                augmented_df = pd.DataFrame(augmented)
                features_df = pd.concat([features_df, augmented_df], ignore_index=True)
                print(f"   ✅ Augmented to {len(features_df)} samples")
    
    # Train model
    print("\n🎯 Training models...")
    trainer = DeadlockModelTrainer()
    
    X, y = trainer.prepare_data(features_df)
    
    # Cross-validation
    cv_results = trainer.cross_validate(X, y, cv=args.cv)
    
    # Full training
    results = trainer.train(X, y)
    
    # Print detailed results
    print("\n" + "=" * 60)
    print("📊 Detailed Results")
    print("=" * 60)
    
    for name, result in results.items():
        print(f"\n{name}:")
        print(f"   Confusion Matrix:")
        cm = confusion_matrix(result['y_test'], result['y_pred'])
        print(f"   {cm}")
        print(f"\n   Classification Report:")
        print(classification_report(result['y_test'], result['y_pred'], target_names=['Normal', 'Deadlock']))
    
    # Save model
    trainer.save_model(args.output)
    
    # Feature importance (for tree-based models)
    if hasattr(trainer.best_model, 'feature_importances_'):
        print("\n📊 Feature Importance:")
        importance = trainer.best_model.feature_importances_
        indices = np.argsort(importance)[::-1]
        
        for i in range(min(10, len(indices))):
            idx = indices[i]
            print(f"   {trainer.feature_columns[idx]}: {importance[idx]:.4f}")
    
    print("\n" + "=" * 60)
    print("✅ Training complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()

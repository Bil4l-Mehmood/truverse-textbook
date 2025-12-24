import React from 'react';
import { HardwareSpec } from '../../services/skillsService';
import styles from './SkillsCards.module.css';

interface HardwareSpecCardProps {
  spec: HardwareSpec;
}

export default function HardwareSpecCard({ spec }: HardwareSpecCardProps) {
  if (spec.error) {
    return (
      <div className={styles.errorCard}>
        <div className={styles.errorIcon}>⚠️</div>
        <p className={styles.errorMessage}>{spec.error}</p>
        {spec.suggestion && <p className={styles.suggestion}>{spec.suggestion}</p>}
      </div>
    );
  }

  return (
    <div className={styles.hardwareCard}>
      <div className={styles.cardHeader}>
        <span className={styles.icon}>🖥️</span>
        <h3 className={styles.componentName}>{spec.component}</h3>
      </div>

      {spec.note && <p className={styles.note}>{spec.note}</p>}

      <div className={styles.specsGrid}>
        <div className={styles.specItem}>
          <label>CPU</label>
          <p>{spec.cpu}</p>
        </div>

        <div className={styles.specItem}>
          <label>GPU</label>
          <p>{spec.gpu}</p>
        </div>

        <div className={styles.specItem}>
          <label>RAM</label>
          <p>{spec.ram}</p>
        </div>

        <div className={styles.specItem}>
          <label>Power</label>
          <p>{spec.power_consumption}</p>
        </div>

        <div className={styles.specItem}>
          <label>Price Range</label>
          <p className={styles.price}>{spec.price_range}</p>
        </div>

        <div className={styles.specItem}>
          <label>Availability</label>
          <p>{spec.availability}</p>
        </div>
      </div>

      {spec.use_cases && spec.use_cases.length > 0 && (
        <div className={styles.useCases}>
          <label>Use Cases</label>
          <ul>
            {spec.use_cases.map((useCase, index) => (
              <li key={index}>✓ {useCase}</li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}

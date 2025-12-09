import React from 'react';
import clsx from 'clsx';
import styles from './BookNote.module.css';

const BookNoteList = [
  {
    title: 'Learning Objective',
    content: 'Understand key concepts',
    icon: '🎓',
  },
  {
    title: 'Exercise',
    content: 'Apply learned concepts',
    icon: '✍️',
  },
  {
    title: 'Example',
    content: 'See concept in action',
    icon: '💡',
  },
];

function Note({title, content, icon}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{icon} {title}</h3>
        <p>{content}</p>
      </div>
    </div>
  );
}

export default function BookNotes() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {BookNoteList.map((props, idx) => (
            <Note key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
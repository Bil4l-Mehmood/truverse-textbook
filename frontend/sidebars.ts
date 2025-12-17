import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    'quarter-overview',
    'hardware-requirements',
    'learning-outcomes',
    {
      type: 'category',
      label: 'Course Modules',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Weeks 1-2: ROS 2 Foundations',
          collapsed: true,
          items: [
            'weeks-01-02/index',
          ],
        },
        {
          type: 'category',
          label: 'Weeks 3-4: Computer Vision & Perception',
          collapsed: true,
          items: [
            'weeks-03-04/index',
          ],
        },
        {
          type: 'category',
          label: 'Weeks 5-6: Manipulation & Control',
          collapsed: true,
          items: [
            {
              type: 'doc',
              id: 'placeholder',
              label: 'Coming Soon',
              key: 'coming-soon-weeks-5-6',
            },
          ],
        },
        {
          type: 'category',
          label: 'Weeks 7-8: Locomotion & Simulation',
          collapsed: true,
          items: [
            {
              type: 'doc',
              id: 'placeholder',
              label: 'Coming Soon',
              key: 'coming-soon-weeks-7-8',
            },
          ],
        },
        {
          type: 'category',
          label: 'Weeks 9-10: Foundation Models & Integration',
          collapsed: true,
          items: [
            {
              type: 'doc',
              id: 'placeholder',
              label: 'Coming Soon',
              key: 'coming-soon-weeks-9-10',
            },
          ],
        },
      ],
    },
  ],
};

export default sidebars;

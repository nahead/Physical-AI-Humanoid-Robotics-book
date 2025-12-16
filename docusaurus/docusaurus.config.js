// @ts-check
import { themes as prismThemes } from "prism-react-renderer";

// Load environment variables from .env file
require('dotenv').config();

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "An educational textbook for the modern robotics engineer.",
  favicon: "img/favicon.ico",

  // Add custom fields for environment variables
  customFields: {
    apiBaseUrl: process.env.NEXT_PUBLIC_API_BASE_URL,
    supabaseUrl: process.env.NEXT_PUBLIC_SUPABASE_URL,
    supabaseAnonKey: process.env.NEXT_PUBLIC_SUPABASE_ANON_KEY,
  },

  // ✅ Vercel production URL
  url: "https://physical-ai-humanoid-roboticsbook.vercel.app",

  // ✅ Vercel always serves from root
  baseUrl: "/",

  // GitHub repo info (for Edit button)
  organizationName: "nahead",
  projectName: "Physical-AI-Humanoid-Robotics-book",

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en", "ur"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: "./sidebars.js",
          editUrl:
            "https://github.com/nahead/Physical-AI-Humanoid-Robotics-book/tree/main/",
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: "img/docusaurus-social-card.jpg",
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: "Physical AI & Humanoid Robotics",
        logo: {
          alt: "Book Logo",
          src: "img/logo.svg",
        },
        items: [
          {
            type: "docSidebar",
            sidebarId: "bookSidebar",
            position: "left",
            label: "Book",
          },
          {
            to: '/login', // Standard link to the login page
            label: 'Login',
            position: 'right',
          },
          {
            href: "https://github.com/nahead/Physical-AI-Humanoid-Robotics-book",
            label: "GitHub",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        links: [
          {
            title: "Book",
            items: [
              {
                label: "Introduction",
                to: "/docs/module1-ros/intro-to-ros",
              },
            ],
          },
          {
            title: "Community",
            items: [
              {
                label: "Discord",
                href: "https://discordapp.com/invite/docusaurus",
              },
            ],
          },
          {
            title: "More",
            items: [
              {
                label: "GitHub",
                href: "https://github.com/nahead/Physical-AI-Humanoid-Robotics-book",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
